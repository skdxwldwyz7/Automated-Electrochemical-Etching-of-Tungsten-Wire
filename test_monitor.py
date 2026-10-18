"""Record K2400 current during a tungsten-wire etching test.

This script is intentionally a data collector. It does not decide when the
wire has broken and it does not turn the output off because of a current drop.
The output is turned off when the requested monitoring time expires or when
the operator presses Ctrl+C.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import pyvisa
import serial


# Defaults copied from Auto_etching_code.py where applicable.
DEFAULT_MOTOR_PORT = "COM5"
DEFAULT_K2400_ADDR = "GPIB0::24::INSTR"
PULSES_PER_MM = 3200


def send_motor_cmd(ser: serial.Serial, pulses: int, direction: int = 0, speed: int = 1000) -> None:
    """Send the same relative-position command used by the original script."""
    if pulses < 0:
        raise ValueError("pulses must be non-negative")
    if not 0 <= direction <= 0xFF:
        raise ValueError("direction must fit in one byte")
    if not 0 < speed <= 0xFFFF:
        raise ValueError("speed must be in the range 1..65535")

    address = 0x01
    command = 0xFD
    speed_bytes = int(speed).to_bytes(2, "big")
    acceleration = 0x0A
    pulse_bytes = int(pulses).to_bytes(4, "big")
    relative_mode = 0x00
    synchronize = 0x00
    checksum = 0x6B
    payload = (
        bytes([address, command, direction])
        + speed_bytes
        + bytes([acceleration])
        + pulse_bytes
        + bytes([relative_mode, synchronize, checksum])
    )
    ser.write(payload)


def parse_current(raw_data: str) -> float:
    """Parse a current value in amperes from a K2400 response."""
    fields = [field.strip() for field in raw_data.strip().split(",")]
    if not fields or not fields[0]:
        raise ValueError(f"empty K2400 response: {raw_data!r}")
    # With ``:FORM:ELEM CURR`` the response is normally one field. Keep the
    # original script's two-field fallback for sessions where the instrument
    # retains a broader response format.
    value_text = fields[0] if len(fields) == 1 else fields[1]
    value = float(value_text)
    if not math.isfinite(value):
        raise ValueError(f"non-finite K2400 response: {raw_data!r}")
    return value


def read_current(device: pyvisa.resources.MessageBasedResource) -> tuple[Optional[float], float, str]:
    """Return (current_a, query_duration_s, error_text).

    A failed query is represented by None, never by zero. This distinction is
    important because zero current is a possible physical result after a wire
    breaks, while a communication failure is not a measurement.
    """
    started = time.perf_counter()
    try:
        raw_data = device.query(":MEAS:CURR?")
        return parse_current(raw_data), time.perf_counter() - started, ""
    except Exception as exc:  # Hardware exceptions vary by VISA backend.
        return None, time.perf_counter() - started, f"{type(exc).__name__}: {exc}"


class CsvLogger:
    """Write every query to disk immediately enough to survive Ctrl+C."""

    def __init__(self, path: Path, run_start: float) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.run_start = run_start
        self.file = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            [
                "sample_index",
                "elapsed_s",
                "wall_time",
                "current_A",
                "current_mA",
                "query_ms",
                "phase",
                "status",
                "error",
            ]
        )
        self.file.flush()
        self.sample_index = 0
        self.phase_counts: dict[str, int] = {}
        self.plot_samples: list[tuple[float, Optional[float], str]] = []

    def record(
        self,
        phase: str,
        current_a: Optional[float],
        query_duration_s: float,
        error: str = "",
    ) -> float:
        elapsed = time.monotonic() - self.run_start
        wall_time = datetime.now().astimezone().isoformat(timespec="milliseconds")
        status = "ok" if current_a is not None else "error"
        current_text = "" if current_a is None else f"{current_a:.12g}"
        current_ma_text = "" if current_a is None else f"{current_a * 1000.0:.9g}"
        self.writer.writerow(
            [
                self.sample_index,
                f"{elapsed:.9f}",
                wall_time,
                current_text,
                current_ma_text,
                f"{query_duration_s * 1000.0:.3f}",
                phase,
                status,
                error,
            ]
        )
        self.file.flush()
        self.plot_samples.append(
            (elapsed, None if current_a is None else current_a * 1000.0, phase)
        )
        self.phase_counts[phase] = self.phase_counts.get(phase, 0) + 1
        self.sample_index += 1
        return elapsed

    def close(self) -> None:
        if not self.file.closed:
            self.file.flush()
            self.file.close()


def configure_k2400(device: pyvisa.resources.MessageBasedResource, voltage: float, nplc: Optional[float]) -> None:
    """Use the same minimal K2400 setup as the known-working original code."""
    # Do not call clear(), change VISA terminations, or add trigger-model
    # commands here. Older K2400/GPIB combinations are sensitive to those
    # commands. The original program only needed these commands.
    # Keep the VISA timeout/measurement defaults used by the original script.
    # A K2400 that is already wedged should not be hammered with retries.
    device.write(":FORM:ELEM CURR")
    device.write(f":SOUR:VOLT {voltage:.9g}")
    if nplc is not None:
        device.write(f":SENS:CURR:NPLC {nplc:.9g}")
    device.write(":OUTP ON")


def measure_and_log(
    device: pyvisa.resources.MessageBasedResource,
    logger: CsvLogger,
    phase: str,
) -> tuple[Optional[float], float, str]:
    current_a, query_duration, error = read_current(device)
    logger.record(phase, current_a, query_duration, error)
    return current_a, query_duration, error


def wait_for_liquid(
    device: pyvisa.resources.MessageBasedResource,
    motor: serial.Serial,
    logger: CsvLogger,
    args: argparse.Namespace,
) -> dict[str, float]:
    """Move down in small increments until a current rise is confirmed.

    The threshold is relative to an air baseline, with an absolute minimum as
    a second guard. Both values are command-line options because the current
    produced by a 25 um wire can vary substantially between cells.
    """
    print("Step 1: finding the liquid surface...")

    baseline_values: list[float] = []
    baseline_deadline = time.monotonic() + max(2.0, args.baseline_samples * args.search_poll * 3.0)
    while len(baseline_values) < args.baseline_samples and time.monotonic() < baseline_deadline:
        current_a, query_duration, _ = measure_and_log(device, logger, "air_baseline")
        if current_a is not None:
            baseline_values.append(current_a)
        time.sleep(max(0.0, args.search_poll - query_duration))

    if not baseline_values:
        raise RuntimeError("could not obtain any valid air-baseline current readings")

    air_baseline_a = statistics.median(baseline_values)
    absolute_threshold_a = args.liquid_min_ua * 1e-6
    air_noise_a = statistics.pstdev(baseline_values) if len(baseline_values) > 1 else 0.0
    relative_rise_a = max(args.liquid_rise_ua * 1e-6, 5.0 * air_noise_a)
    relative_threshold_a = air_baseline_a + relative_rise_a
    threshold_a = max(absolute_threshold_a, relative_threshold_a)
    print(
        "Air baseline: "
        f"{air_baseline_a * 1e6:.3f} uA; noise: {air_noise_a * 1e6:.3f} uA; "
        f"contact threshold: {threshold_a * 1e6:.3f} uA"
    )

    search_start = time.monotonic()
    next_move = search_start
    moved_pulses = 0
    confirmed = 0
    consecutive_errors = 0
    last_current_a: Optional[float] = None

    while time.monotonic() - search_start < args.search_timeout:
        current_a, query_duration, error = measure_and_log(device, logger, "liquid_search")
        last_current_a = current_a
        if current_a is None:
            consecutive_errors += 1
            if consecutive_errors >= args.max_query_errors:
                raise RuntimeError(
                    f"{consecutive_errors} consecutive K2400 query failures; last error: {error}"
                )
        else:
            consecutive_errors = 0
            if current_a >= threshold_a:
                confirmed += 1
            else:
                confirmed = 0
            if confirmed >= args.liquid_confirm_samples:
                contact_elapsed = time.monotonic() - logger.run_start
                print(
                    f"Liquid contact confirmed at {contact_elapsed:.3f} s; "
                    f"current {current_a * 1e6:.3f} uA"
                )
                return {
                    "elapsed_s": contact_elapsed,
                    "air_baseline_a": air_baseline_a,
                    "threshold_a": threshold_a,
                    "contact_current_a": current_a,
                }

            # Only move after a valid low-current reading. This prevents an
            # unnecessary extra downward step if the wire is already touching
            # the liquid when the script starts.
            now = time.monotonic()
            if now >= next_move:
                if moved_pulses + args.contact_step_pulses > round(args.max_search_mm * PULSES_PER_MM):
                    raise TimeoutError(
                        f"liquid surface not found within {args.max_search_mm:.3f} mm of search travel"
                    )
                send_motor_cmd(
                    motor,
                    args.contact_step_pulses,
                    direction=0,
                    speed=args.contact_speed,
                )
                moved_pulses += args.contact_step_pulses
                next_move = now + args.contact_move_interval
                print(
                    f"  moving down {args.contact_step_pulses} pulses "
                    f"(total {moved_pulses / PULSES_PER_MM:.4f} mm); "
                    f"elapsed {time.monotonic() - search_start:.1f} s"
                )

        time.sleep(max(0.0, args.search_poll - query_duration))

    raise TimeoutError(
        "liquid surface was not confirmed before search timeout; "
        f"last current was {last_current_a!r} A"
    )


def sample_for_duration(
    device: pyvisa.resources.MessageBasedResource,
    logger: CsvLogger,
    duration_s: Optional[float],
    interval_s: float,
    phase: str,
    max_query_errors: int,
) -> float:
    """Acquire current samples for a fixed duration without any drop logic."""
    started = time.monotonic()
    deadline = None if duration_s is None else started + duration_s
    next_sample = started
    consecutive_errors = 0

    while deadline is None or time.monotonic() < deadline:
        if interval_s > 0:
            sleep_time = next_sample - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

        _, query_duration, error = measure_and_log(device, logger, phase)
        if error:
            consecutive_errors += 1
            if consecutive_errors >= max_query_errors:
                raise RuntimeError(
                    f"{consecutive_errors} consecutive K2400 query failures; last error: {error}"
                )
        else:
            consecutive_errors = 0

        if interval_s > 0:
            next_sample += interval_s
            if next_sample < time.monotonic() - interval_s:
                next_sample = time.monotonic()

    return time.monotonic() - started


def save_plot(logger: CsvLogger, path: Path, contact_elapsed_s: Optional[float], monitor_elapsed_s: Optional[float]) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"Warning: plot was not generated because matplotlib is unavailable: {exc}")
        return

    valid_samples = [sample for sample in logger.plot_samples if sample[1] is not None]
    if not valid_samples:
        print("Warning: plot was not generated because no valid current samples were recorded")
        return

    x_values = [sample[0] for sample in valid_samples]
    y_values = [sample[1] for sample in valid_samples]
    figure, axis = plt.subplots(figsize=(11, 6))
    axis.plot(x_values, y_values, linewidth=0.8, color="tab:blue")
    if contact_elapsed_s is not None:
        axis.axvline(contact_elapsed_s, color="tab:green", linestyle="--", label="liquid contact")
    if monitor_elapsed_s is not None:
        axis.axvline(monitor_elapsed_s, color="tab:orange", linestyle="--", label="monitor start")
    axis.set_title("K2400 current during tungsten-wire etching test")
    axis.set_xlabel("Elapsed time (s)")
    axis.set_ylabel("Current (mA)")
    axis.grid(True, linestyle="--", alpha=0.15)
    if contact_elapsed_s is not None or monitor_elapsed_s is not None:
        axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor-port", default=DEFAULT_MOTOR_PORT)
    parser.add_argument("--k2400-addr", default=DEFAULT_K2400_ADDR)
    parser.add_argument("--voltage", type=float, default=5.0)
    parser.add_argument(
        "--nplc",
        type=float,
        default=None,
        help="optional K2400 current integration time; omitted uses the instrument's existing setting",
    )
    parser.add_argument(
        "--sample-interval",
        type=float,
        default=0.30,
        help="monitor sample interval in seconds; default 0.30 s (about 3 Hz)",
    )
    parser.add_argument("--target-depth-mm", type=float, default=1.0)
    parser.add_argument("--immersion-speed", type=int, default=1500)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument(
        "--monitor-seconds",
        type=float,
        default=0.0,
        help="monitor duration in seconds; 0 means run until Ctrl+C",
    )
    parser.add_argument("--search-timeout", type=float, default=60.0)
    parser.add_argument("--max-search-mm", type=float, default=5.0)
    parser.add_argument("--baseline-samples", type=int, default=20)
    parser.add_argument("--search-poll", type=float, default=0.30)
    parser.add_argument("--contact-step-pulses", type=int, default=160)
    parser.add_argument("--contact-speed", type=int, default=1000)
    parser.add_argument("--contact-move-interval", type=float, default=0.35)
    parser.add_argument(
        "--liquid-min-ua",
        type=float,
        default=0.0,
        help="absolute liquid threshold in uA; 0 uses only the baseline-relative threshold",
    )
    parser.add_argument("--liquid-rise-ua", type=float, default=0.2)
    parser.add_argument("--liquid-confirm-samples", type=int, default=3)
    parser.add_argument(
        "--max-query-errors",
        type=int,
        default=1,
        help="stop after this many consecutive VISA query errors; default 1 for instrument safety",
    )
    parser.add_argument("--retract-mm", type=float, default=5.0)
    parser.add_argument("--retract-speed", type=int, default=1500)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "test_monitor_data",
        help="directory for CSV, JSON and PNG files",
    )
    parser.add_argument("--no-retract", action="store_true", help="do not move the motor upward during cleanup")
    parser.add_argument("--no-plot", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.nplc is not None and args.nplc <= 0:
        raise SystemExit("nplc must be positive when supplied")
    if args.monitor_seconds < 0 or args.target_depth_mm <= 0 or args.max_search_mm <= 0:
        raise SystemExit("target-depth-mm and max-search-mm must be positive; monitor-seconds must be >= 0")
    if args.baseline_samples <= 0 or args.liquid_confirm_samples <= 0:
        raise SystemExit("baseline-samples and liquid-confirm-samples must be positive")
    if args.sample_interval <= 0 or args.search_poll <= 0 or args.max_query_errors <= 0:
        raise SystemExit("sample-interval and search-poll must be positive")
    if args.sample_interval < 0.10:
        print("Warning: sample-interval below 0.10 s can overload a K2400/GPIB link")

    run_start = time.monotonic()
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / f"etch_test_{stamp}.csv"
    json_path = args.output_dir / f"etch_test_{stamp}.json"
    plot_path = args.output_dir / f"etch_test_{stamp}.png"
    logger = CsvLogger(csv_path, run_start)

    metadata: dict[str, object] = {
        "started_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "csv_file": str(csv_path),
        "plot_file": None if args.no_plot else str(plot_path),
        "config": {key: str(value) for key, value in vars(args).items()},
        "contact": None,
        "monitor_start_elapsed_s": None,
        "monitor_duration_s": None,
        "stop_reason": None,
        "error": None,
    }

    resource_manager = None
    k2400 = None
    motor = None
    contact_info: Optional[dict[str, float]] = None
    monitor_start_elapsed: Optional[float] = None
    exit_code = 0

    try:
        resource_manager = pyvisa.ResourceManager()
        k2400 = resource_manager.open_resource(args.k2400_addr)
        motor = serial.Serial(args.motor_port, 115200, timeout=0.1)
        configure_k2400(k2400, args.voltage, args.nplc)

        print("K2400 output is ON. This run records data and does not auto-stop on current drop.")
        contact_info = wait_for_liquid(k2400, motor, logger, args)
        metadata["contact"] = contact_info

        immersion_pulses = round(args.target_depth_mm * PULSES_PER_MM)
        print(f"Step 2: moving {args.target_depth_mm:.4f} mm into the liquid...")
        send_motor_cmd(motor, immersion_pulses, direction=0, speed=args.immersion_speed)
        sample_for_duration(
            k2400,
            logger,
            args.settle_seconds,
            max(args.sample_interval, 0.10),
            "immersion_settle",
            args.max_query_errors,
        )

        monitor_start_elapsed = time.monotonic() - run_start
        metadata["monitor_start_elapsed_s"] = monitor_start_elapsed
        monitor_limit = None if args.monitor_seconds == 0 else args.monitor_seconds
        monitor_description = "until Ctrl+C" if monitor_limit is None else f"for {monitor_limit:.1f} s"
        print(f"Step 3: monitoring {monitor_description}. Press Ctrl+C after the desired over-etch region is visible.")
        monitor_duration = sample_for_duration(
            k2400,
            logger,
            monitor_limit,
            args.sample_interval,
            "monitor",
            args.max_query_errors,
        )
        metadata["monitor_duration_s"] = monitor_duration
        metadata["stop_reason"] = "monitor_timeout"
    except KeyboardInterrupt:
        metadata["stop_reason"] = "keyboard_interrupt"
        print("Stopping by operator request...")
    except Exception as exc:
        exit_code = 1
        metadata["stop_reason"] = "error"
        metadata["error"] = f"{type(exc).__name__}: {exc}"
        print(f"Error: {type(exc).__name__}: {exc}")
    finally:
        if k2400 is not None:
            try:
                k2400.write(":OUTP OFF")
                print("K2400 output OFF.")
            except Exception as exc:
                print(f"Warning: could not turn K2400 output off: {exc}")

        if motor is not None and not args.no_retract:
            try:
                retract_pulses = round(args.retract_mm * PULSES_PER_MM)
                if retract_pulses > 0:
                    send_motor_cmd(
                        motor,
                        retract_pulses,
                        direction=1,
                        speed=args.retract_speed,
                    )
                    print(f"Retracting motor by {args.retract_mm:.3f} mm.")
            except Exception as exc:
                exit_code = 1
                print(f"Warning: could not retract motor: {exc}")

        if k2400 is not None:
            try:
                k2400.close()
            except Exception:
                pass
        if motor is not None:
            try:
                motor.close()
            except Exception:
                pass
        if resource_manager is not None:
            try:
                resource_manager.close()
            except Exception:
                pass

        logger.close()
        if not args.no_plot:
            save_plot(logger, plot_path, None if contact_info is None else contact_info["elapsed_s"], monitor_start_elapsed)

        metadata["finished_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
        metadata["sample_count"] = logger.sample_index
        metadata["valid_sample_count"] = sum(sample[1] is not None for sample in logger.plot_samples)
        metadata["sample_counts_by_phase"] = logger.phase_counts
        if monitor_start_elapsed is not None:
            elapsed = max(0.0, time.monotonic() - run_start - monitor_start_elapsed)
            metadata["observed_monitor_rate_hz"] = (
                logger.phase_counts.get("monitor", 0) / elapsed if elapsed > 0 else None
            )
        json_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        print(f"CSV:  {csv_path}")
        print(f"JSON: {json_path}")
        if not args.no_plot:
            print(f"Plot: {plot_path}")

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
