"""初版钨丝电化学刻蚀控制 GUI。

本程序把原来的 ``Auto_etching_code.py`` 流程接入 PyQt5 图形界面，同时提供：

* X42 电机的串口连接、相对/绝对位置模式和手动发送指令；
* Keithley 2400 的 VISA 连接、电压、量程、输出和实时电流读取；
* 找液面、下探、刻蚀监控、断点判断、断电和回撤；
* 实时电流曲线，以及可选的 CSV、参数 JSON 和 PNG 保存。

运行环境需要 ``PyQt5``、``pyvisa``、``pyserial`` 和 ``matplotlib``。
硬件连接地址和参数均可在界面中修改，默认值沿用原自动刻蚀脚本。
"""

from __future__ import annotations

import csv
import json
import math
import statistics
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

try:
    import pyvisa
    import serial
    from serial.tools import list_ports
except Exception:  # pragma: no cover - gives a useful GUI error when installed incompletely
    pyvisa = None
    serial = None
    list_ports = None

from PyQt5.QtCore import QObject, QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QDoubleValidator, QIntValidator
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

try:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
except Exception:  # pragma: no cover
    FigureCanvas = None
    Figure = None


APP_DIR = Path(__file__).resolve().parent
DEFAULT_SAVE_DIR = APP_DIR / "etch_gui_data"
PULSES_PER_MM = 3200


def now_stamp() -> str:
    return datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")


def parse_current(raw: str) -> float:
    """Parse the current in amperes from a K2400 response."""
    fields = [part.strip() for part in raw.strip().split(",") if part.strip()]
    if not fields:
        raise ValueError(f"K2400 返回为空: {raw!r}")
    # :FORM:ELEM CURR usually returns one value. Keep the original script's
    # two-field fallback for a device retaining a broader response format.
    value = float(fields[0] if len(fields) == 1 else fields[1])
    if not math.isfinite(value) or abs(value) >= 9e37:
        raise ValueError(f"K2400 返回无效或溢出读数: {raw!r}")
    return value


class MotorController:
    """X42-style motor serial protocol used by the original script."""

    def __init__(self) -> None:
        self.ser: Any = None

    @property
    def connected(self) -> bool:
        return self.ser is not None and bool(getattr(self.ser, "is_open", False))

    def connect(self, port: str, baud: int, timeout: float = 0.1) -> None:
        if serial is None:
            raise RuntimeError("未安装 pyserial")
        self.close()
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)

    def close(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            finally:
                self.ser = None

    def send_position(
        self,
        pulses: int,
        direction: int = 0,
        speed: int = 3000,
        acceleration: int = 10,
        relative: bool = True,
        synchronize: bool = False,
    ) -> None:
        if not self.connected:
            raise RuntimeError("电机串口尚未打开")
        if pulses < 0 or pulses > 0xFFFFFFFF:
            raise ValueError("脉冲数必须在 0..4294967295 范围内")
        if not 1 <= speed <= 0xFFFF:
            raise ValueError("速度必须在 1..65535 范围内")
        if not 0 <= acceleration <= 0xFF:
            raise ValueError("加速度必须在 0..255 范围内")
        payload = (
            bytes([0x01, 0xFD, direction & 0xFF])
            + int(speed).to_bytes(2, "big")
            + bytes([acceleration & 0xFF])
            + int(pulses).to_bytes(4, "big")
            + bytes([0x00 if relative else 0x01, 0x01 if synchronize else 0x00, 0x6B])
        )
        self.ser.write(payload)


class K2400Controller:
    def __init__(self) -> None:
        self.rm: Any = None
        self.device: Any = None
        self.identity = ""
        self._lock = threading.RLock()

    @property
    def connected(self) -> bool:
        return self.device is not None

    def resources(self) -> list[str]:
        if pyvisa is None:
            return []
        rm = None
        try:
            rm = pyvisa.ResourceManager()
            return list(rm.list_resources())
        except Exception:
            return []
        finally:
            if rm is not None:
                try:
                    rm.close()
                except Exception:
                    pass

    def connect(self, address: str) -> None:
        if pyvisa is None:
            raise RuntimeError("未安装 pyvisa")
        self.close()
        try:
            self.rm = pyvisa.ResourceManager()
            self.device = self.rm.open_resource(address.strip())
            self.device.timeout = 3000
            self.device.write_termination = "\n"
            self.device.read_termination = "\n"
            if address.strip().upper().startswith("GPIB"):
                self.device.control_ren(pyvisa.constants.RENLineOperation.asrt_address)
            self.identity = self.query("*IDN?").strip()
            if not any(part.strip().upper() == "MODEL 2400" for part in self.identity.split(",")):
                raise RuntimeError(f"所选地址不是 Keithley 2400：{self.identity!r}")
            self.write(":ABOR")
            self.write(":OUTP OFF")
            # Report existing errors rather than silently discarding them with *CLS.
            self.check_errors("连接初始化（可能含仪器原有错误）")
            self.prepare_measurement()
            self.check_errors("初始化电流测量")
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        with self._lock:
            self._close_unlocked()

    def _close_unlocked(self) -> None:
        if self.device is not None:
            try:
                self.device.write(":ABOR")
                self.device.write(":OUTP OFF")
            except Exception:
                pass
            try:
                self.device.close()
            except Exception:
                pass
        self.device = None
        if self.rm is not None:
            try:
                self.rm.close()
            except Exception:
                pass
        self.rm = None
        self.identity = ""

    def write(self, command: str) -> None:
        with self._lock:
            if not self.connected:
                raise RuntimeError("K2400 尚未连接")
            self.device.write(command)

    def query(self, command: str) -> str:
        with self._lock:
            if not self.connected:
                raise RuntimeError("K2400 尚未连接")
            return self.device.query(command)

    def check_errors(self, operation: str) -> None:
        errors = []
        for _ in range(20):
            reply = self.query(":SYST:ERR?").strip()
            if int(reply.split(",", 1)[0]) == 0:
                break
            errors.append(reply)
        if errors:
            raise RuntimeError(f"{operation}：K2400 错误队列：" + "; ".join(errors))

    def prepare_measurement(self) -> None:
        # MEAS:CURR? implicitly executes CONFigure, resets ranges and turns ON
        # the output. Configure explicitly once, then acquire with READ?.
        for command in (
            ":SOUR:FUNC VOLT", ":SOUR:VOLT:MODE FIX",
            ":SOUR:VOLT:RANG:AUTO ON", ":SOUR:CLE:AUTO OFF",
            ':SENS:FUNC:CONC OFF', ':SENS:FUNC "CURR"',
            ":FORM:DATA ASC", ":FORM:ELEM CURR",
            ":ARM:SOUR IMM", ":ARM:COUN 1", ":TRIG:SOUR IMM",
            ":TRIG:COUN 1", ":TRIG:DEL 0", ":TRAC:FEED:CONT NEV",
        ):
            self.write(command)

    def set_voltage(self, voltage: float) -> None:
        self.write(":SOUR:FUNC VOLT")
        self.write(":SOUR:VOLT:MODE FIX")
        self.write(":SOUR:VOLT:RANG:AUTO ON")
        self.write(f":SOUR:VOLT {float(voltage):.9g}")

    def set_range(self, range_value: Optional[float]) -> None:
        with self._lock:
            if range_value is None:
                self.write(":SENS:CURR:RANG:AUTO ON")
            else:
                self.write(":SENS:CURR:RANG:AUTO OFF")
                self.write(f":SENS:CURR:RANG {float(range_value):.9g}")
            self.check_errors("设置电流量程（量程上限受电流限值和源电压档位限制）")
            auto = bool(int(float(self.query(":SENS:CURR:RANG:AUTO?"))))
            actual = float(self.query(":SENS:CURR:RANG?"))
            if auto != (range_value is None) or (
                range_value is not None and not (
                    range_value * 0.9999 <= actual <= range_value * 1.0501
                )
            ):
                raise RuntimeError(f"电流量程回读不一致：AUTO={auto}, RANGE={actual:g} A")

    def output(self, enabled: bool) -> None:
        with self._lock:
            self.write(":OUTP ON" if enabled else ":OUTP OFF")
            self.check_errors("切换输出")
            actual = bool(int(float(self.query(":OUTP?"))))
            if actual != enabled:
                raise RuntimeError(f"输出状态回读不一致：请求 {enabled}，实际 {actual}")

    def configure(
        self, voltage: float, range_value: Optional[float], current_limit_a: Optional[float] = None
    ) -> None:
        with self._lock:
            self.set_voltage(voltage)
            if current_limit_a is not None:
                self.write(f":SENS:CURR:PROT {float(current_limit_a):.9g}")
                self.check_errors("设置电流限值")
                actual_limit = float(self.query(":SENS:CURR:PROT?"))
                if not math.isclose(actual_limit, current_limit_a, rel_tol=1e-4, abs_tol=1e-9):
                    raise RuntimeError(f"电流限值回读不一致：{actual_limit:g} A")
            self.set_range(range_value)
            self.check_errors("应用电压/量程")
            actual = float(self.query(":SOUR:VOLT?"))
            if not math.isclose(actual, voltage, rel_tol=1e-5, abs_tol=1e-6):
                raise RuntimeError(f"电压回读不一致：请求 {voltage:g} V，实际 {actual:g} V")

    def current_a(self) -> float:
        with self._lock:
            if not bool(int(float(self.query(":OUTP?")))):
                raise RuntimeError("输出为 OFF，未采集电流；请先手动开启输出")
            try:
                return parse_current(self.query(":READ?"))
            except Exception:
                # Abort an unfinished trigger; do not let a late response be
                # mistaken for a later query. Disconnect/reconnect after timeout.
                try:
                    self.write(":ABOR")
                    self.write(":OUTP OFF")
                except Exception:
                    pass
                raise


class EtchWorker(QObject):
    """Long-running hardware sequence, kept off the GUI thread."""

    log = pyqtSignal(str)
    sample = pyqtSignal(float, float, str)  # elapsed_s, current_mA, phase
    phase = pyqtSignal(str)
    finished = pyqtSignal(object)

    def __init__(
        self,
        motor: MotorController,
        k2400: K2400Controller,
        params: dict[str, Any],
        paths: dict[str, Optional[str]],
    ) -> None:
        super().__init__()
        self.motor = motor
        self.k2400 = k2400
        self.params = params
        self.paths = paths
        self.stop_event = threading.Event()
        self.start_time = 0.0
        self.rows: list[dict[str, Any]] = []
        self.times: list[float] = []
        self.currents: list[float] = []
        self.contact_elapsed: Optional[float] = None
        self.monitor_start_elapsed: Optional[float] = None

    def request_stop(self) -> None:
        self.stop_event.set()

    def stopped(self) -> bool:
        return self.stop_event.is_set()

    def wait_interruptible(self, seconds: float) -> bool:
        return not self.stop_event.wait(max(0.0, seconds))

    def send_move(self, pulses: int, direction: int, speed: int) -> None:
        self.motor.send_position(
            int(abs(pulses)),
            direction=direction,
            speed=int(speed),
            acceleration=int(self.params["acceleration"]),
            relative=True,
            synchronize=False,
        )

    def record(self, phase: str, current_a: Optional[float], error: str = "") -> Optional[float]:
        elapsed = time.monotonic() - self.start_time
        current_ma = None if current_a is None else current_a * 1000.0
        row = {
            "sample_index": len(self.rows),
            "elapsed_s": elapsed,
            "wall_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "current_A": current_a,
            "current_mA": current_ma,
            "phase": phase,
            "status": "ok" if current_a is not None else "error",
            "error": error,
        }
        self.rows.append(row)
        if current_ma is not None and math.isfinite(current_ma):
            self.times.append(elapsed)
            self.currents.append(current_ma)
            self.sample.emit(elapsed, current_ma, phase)
        return current_a

    def query_and_record(self, phase: str) -> Optional[float]:
        started = time.perf_counter()
        try:
            value = self.k2400.current_a()
            self.record(phase, value)
            return value
        except Exception as exc:
            duration_ms = (time.perf_counter() - started) * 1000.0
            message = f"{type(exc).__name__}: {exc} (查询耗时 {duration_ms:.1f} ms)"
            self.record(phase, None, message)
            self.log.emit("K2400 电流读取失败: " + message)
            return None

    def write_csv(self) -> None:
        path = self.paths.get("csv")
        if not path:
            return
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("w", newline="", encoding="utf-8-sig") as file:
            writer = csv.writer(file)
            writer.writerow(
                ["sample_index", "elapsed_s", "wall_time", "current_A", "current_mA", "phase", "status", "error"]
            )
            for row in self.rows:
                writer.writerow(
                    [
                        row["sample_index"],
                        f"{row['elapsed_s']:.9f}",
                        row["wall_time"],
                        "" if row["current_A"] is None else f"{row['current_A']:.12g}",
                        "" if row["current_mA"] is None else f"{row['current_mA']:.9g}",
                        row["phase"],
                        row["status"],
                        row["error"],
                    ]
                )

    def write_json(self, result: dict[str, Any]) -> None:
        path = self.paths.get("json")
        if not path:
            return
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    def run(self) -> None:
        self.start_time = time.monotonic()
        success = False
        reason = ""
        error_text = ""
        consecutive_errors = 0
        try:
            p = self.params
            self.log.emit("自动刻蚀开始：正在配置 K2400…")
            self.k2400.configure(float(p["voltage"]), p["range_value"], p.get("current_limit_a"))
            self.k2400.output(True)

            # Step 1: air baseline and liquid search.
            self.phase.emit("寻找液面")
            self.log.emit("Step 1/3：采集空气基线并寻找液面")
            baseline: list[float] = []
            for _ in range(int(p["baseline_samples"])):
                if self.stopped():
                    raise InterruptedError("用户停止")
                value = self.query_and_record("air_baseline")
                if value is not None:
                    baseline.append(value)
                if not self.wait_interruptible(float(p["search_poll_s"])):
                    raise InterruptedError("用户停止")
            if not baseline:
                raise RuntimeError("没有获得有效的空气基线电流")
            baseline_a = statistics.median(baseline)
            threshold_a = max(
                float(p["liquid_threshold_ma"]) * 1e-3,
                baseline_a + float(p["liquid_rise_ma"]) * 1e-3,
            )
            self.log.emit(
                f"空气基线 {baseline_a * 1000:.4f} mA；液面阈值 {threshold_a * 1000:.4f} mA"
            )
            search_started = time.monotonic()
            moved_pulses = 0
            confirm = 0
            next_move = 0.0
            while time.monotonic() - search_started < float(p["search_timeout_s"]):
                if self.stopped():
                    raise InterruptedError("用户停止")
                value = self.query_and_record("liquid_search")
                if value is None:
                    consecutive_errors += 1
                    if consecutive_errors >= int(p["max_query_errors"]):
                        raise RuntimeError("K2400 连续电流读取失败")
                else:
                    consecutive_errors = 0
                    confirm = confirm + 1 if value >= threshold_a else 0
                    if confirm >= int(p["liquid_confirm_samples"]):
                        self.contact_elapsed = time.monotonic() - self.start_time
                        self.log.emit(f"已确认液面，电流 {value * 1000:.4f} mA")
                        break
                    if time.monotonic() >= next_move:
                        max_pulses = round(float(p["max_search_mm"]) * float(p["pulses_per_mm"]))
                        step = int(p["search_step_pulses"])
                        if moved_pulses + step > max_pulses:
                            raise TimeoutError("在最大搜索行程内没有找到液面")
                        self.send_move(step, 0, int(p["search_speed"]))
                        moved_pulses += step
                        next_move = time.monotonic() + float(p["search_move_interval_s"])
                        self.log.emit(f"电机下移 {step} 脉冲（累计 {moved_pulses / float(p['pulses_per_mm']):.4f} mm）")
                if not self.wait_interruptible(float(p["search_poll_s"])):
                    raise InterruptedError("用户停止")
            else:
                raise TimeoutError("找液面超时")

            # Step 2: immersion.
            self.phase.emit("下探稳定")
            depth_pulses = round(float(p["target_depth_mm"]) * float(p["pulses_per_mm"]))
            self.log.emit(f"Step 2/3：下探 {float(p['target_depth_mm']):.4f} mm")
            self.send_move(depth_pulses, 0, int(p["immersion_speed"]))
            settle_deadline = time.monotonic() + float(p["settle_s"])
            while time.monotonic() < settle_deadline:
                if self.stopped():
                    raise InterruptedError("用户停止")
                self.query_and_record("immersion_settle")
                if not self.wait_interruptible(float(p["sample_interval_s"])):
                    raise InterruptedError("用户停止")

            # Step 3: monitor using the sudden-drop logic from Auto_etching_code.py.
            self.phase.emit("刻蚀监控")
            self.monitor_start_elapsed = time.monotonic() - self.start_time
            self.log.emit("Step 3/3：刻蚀监控中")
            previous: list[float] = []
            break_count = 0
            sudden_count = 0
            sudden_level: Optional[float] = None
            sudden_from: Optional[float] = None
            monitor_started = time.monotonic()
            while True:
                if self.stopped():
                    raise InterruptedError("用户停止")
                value = self.query_and_record("monitor")
                if value is None:
                    consecutive_errors += 1
                    if consecutive_errors >= int(p["max_query_errors"]):
                        raise RuntimeError("K2400 连续电流读取失败")
                    if not self.wait_interruptible(float(p["sample_interval_s"])):
                        raise InterruptedError("用户停止")
                    continue
                consecutive_errors = 0
                current_ma = value * 1000.0
                baseline_ma = statistics.median(previous[-int(p["drop_baseline_samples"]):]) if previous else current_ma
                delta = baseline_ma - current_ma
                elapsed_monitor = time.monotonic() - monitor_started
                if current_ma <= float(p["overetch_current_ma"]):
                    reason = f"电流降至过刻保护值 {current_ma:.4f} mA"
                elif elapsed_monitor >= float(p["min_monitor_s"]):
                    relative_drop = delta / baseline_ma if baseline_ma > 0 else 0.0
                    is_sudden = (
                        baseline_ma >= float(p["min_pre_drop_ma"])
                        and delta >= float(p["sudden_drop_ma"])
                        and relative_drop >= float(p["sudden_drop_ratio"])
                        and current_ma <= float(p["max_drop_result_ma"])
                    )
                    if sudden_count:
                        if sudden_level is not None and current_ma <= sudden_level * (1.0 + float(p["drop_tolerance"])):
                            sudden_count += 1
                        else:
                            sudden_count = 0
                            sudden_level = None
                            sudden_from = None
                    elif is_sudden:
                        sudden_count = 1
                        sudden_level = current_ma
                        sudden_from = baseline_ma
                    if sudden_count >= int(p["drop_confirm_samples"]):
                        reason = f"检测到电流突降：{sudden_from:.4f} -> {sudden_level:.4f} mA"
                if not reason and current_ma <= float(p["break_current_ma"]):
                    break_count += 1
                    if break_count >= int(p["break_confirm_samples"]):
                        reason = f"检测到低电流平台：{current_ma:.4f} mA"
                elif current_ma > float(p["break_current_ma"]):
                    break_count = 0
                previous.append(current_ma)
                if len(previous) > int(p["drop_baseline_samples"]):
                    previous.pop(0)
                if reason:
                    self.log.emit("刻蚀完成：" + reason)
                    break
                monitor_limit = float(p["monitor_limit_s"])
                if monitor_limit > 0 and elapsed_monitor >= monitor_limit:
                    reason = f"达到监控时长 {monitor_limit:.1f} s"
                    self.log.emit(reason)
                    break
                if not self.wait_interruptible(float(p["sample_interval_s"])):
                    raise InterruptedError("用户停止")

            self.k2400.output(False)
            retract_pulses = round(float(p["retract_mm"]) * float(p["pulses_per_mm"]))
            if retract_pulses > 0:
                self.log.emit(f"断电并回撤 {float(p['retract_mm']):.4f} mm")
                self.send_move(retract_pulses, 1, int(p["retract_speed"]))
            success = True
        except InterruptedError as exc:
            reason = str(exc)
            self.log.emit("已停止：" + reason)
        except Exception as exc:
            error_text = f"{type(exc).__name__}: {exc}"
            reason = "异常停止"
            self.log.emit("错误：" + error_text)
        finally:
            try:
                self.k2400.output(False)
            except Exception as exc:
                self.log.emit(f"关闭 K2400 输出失败：{exc}")
            try:
                self.write_csv()
            except Exception as exc:
                error_text = error_text or f"保存 CSV 失败：{exc}"
                self.log.emit(error_text)
            result: dict[str, Any] = {
                "success": success,
                "reason": reason,
                "error": error_text,
                "started_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                "finished_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                "contact_elapsed_s": self.contact_elapsed,
                "monitor_start_elapsed_s": self.monitor_start_elapsed,
                "sample_count": len(self.rows),
                "config": self.params,
                "files": self.paths,
            }
            try:
                self.write_json(result)
            except Exception as exc:
                self.log.emit(f"保存参数 JSON 失败：{exc}")
            self.finished.emit({"result": result, "times": self.times, "currents": self.currents})


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("X42 电机 + Keithley 2400 钨丝电化学刻蚀控制器")
        self.resize(1400, 900)
        self.motor = MotorController()
        self.k2400 = K2400Controller()
        self.thread: Optional[QThread] = None
        self.worker: Optional[EtchWorker] = None
        self.plot_times: list[float] = []
        self.plot_currents: list[float] = []
        self._build_ui()
        self._refresh_ports()
        self._refresh_resources()
        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll_current)
        self.statusBar().showMessage("就绪。请先连接电机串口和 K2400。")

    # ---- small widget helpers -------------------------------------------------
    @staticmethod
    def spin(minimum: int, maximum: int, value: int, suffix: str = "") -> QSpinBox:
        widget = QSpinBox()
        widget.setRange(minimum, maximum)
        widget.setValue(value)
        widget.setSuffix(suffix)
        return widget

    @staticmethod
    def dspin(minimum: float, maximum: float, value: float, decimals: int = 4, suffix: str = "") -> QDoubleSpinBox:
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setDecimals(decimals)
        widget.setValue(value)
        widget.setSuffix(suffix)
        widget.setSingleStep(10 ** (-decimals))
        return widget

    @staticmethod
    def combo(items: list[tuple[str, Any]] | list[str]) -> QComboBox:
        widget = QComboBox()
        if items and isinstance(items[0], tuple):
            for text, data in items:  # type: ignore[misc]
                widget.addItem(text, data)
        else:
            widget.addItems(items)  # type: ignore[arg-type]
        return widget

    @staticmethod
    def row_button(button: QPushButton, layout: QHBoxLayout) -> None:
        layout.addWidget(button)

    def _build_ui(self) -> None:
        root = QWidget()
        root_layout = QHBoxLayout(root)
        splitter = QSplitter(Qt.Horizontal)
        root_layout.addWidget(splitter)

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setMinimumWidth(440)
        left_content = QWidget()
        left_layout = QVBoxLayout(left_content)
        left_scroll.setWidget(left_content)
        splitter.addWidget(left_scroll)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        left_layout.addWidget(self._build_device_group())
        left_layout.addWidget(self._build_motor_group())
        left_layout.addWidget(self._build_k2400_group())
        left_layout.addWidget(self._build_etch_group())
        left_layout.addWidget(self._build_save_group())
        left_layout.addStretch(1)

        plot_group = QGroupBox("实时 K2400 电流")
        plot_layout = QVBoxLayout(plot_group)
        if FigureCanvas is not None:
            self.figure = Figure(figsize=(8, 5), tight_layout=True)
            self.axes = self.figure.add_subplot(111)
            # Keep plot annotations in ASCII/English so Matplotlib's default
            # DejaVu Sans font does not emit missing-CJK-glyph warnings.
            self.axes.set_xlabel("Time (s)")
            self.axes.set_ylabel("Current (mA)")
            self.axes.grid(True, linestyle="--", alpha=0.25)
            self.line, = self.axes.plot([], [], color="#1565c0", linewidth=1.3)
            self.canvas = FigureCanvas(self.figure)
            plot_layout.addWidget(self.canvas)
        else:
            plot_layout.addWidget(QLabel("matplotlib 不可用，无法显示曲线。"))
        right_layout.addWidget(plot_group, 3)

        log_group = QGroupBox("运行日志")
        log_layout = QVBoxLayout(log_group)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(160)
        log_layout.addWidget(self.log_text)
        right_layout.addWidget(log_group, 1)
        self.setCentralWidget(root)
        self.setStatusBar(QStatusBar())

    def _build_device_group(self) -> QWidget:
        group = QGroupBox("设备连接")
        layout = QFormLayout(group)

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        refresh = QPushButton("刷新")
        refresh.clicked.connect(self._refresh_ports)
        port_row = QHBoxLayout()
        port_row.addWidget(self.port_combo, 1)
        port_row.addWidget(refresh)
        layout.addRow("电机串口", port_row)
        self.baud_combo = self.combo(["9600", "38400", "115200", "230400"])
        self.baud_combo.setCurrentText("115200")
        layout.addRow("波特率", self.baud_combo)
        self.motor_connect_btn = QPushButton("打开串口")
        self.motor_connect_btn.clicked.connect(self._toggle_motor)
        layout.addRow("电机", self.motor_connect_btn)

        self.visa_combo = QComboBox()
        self.visa_combo.setEditable(True)
        visa_refresh = QPushButton("扫描")
        visa_refresh.clicked.connect(self._refresh_resources)
        visa_row = QHBoxLayout()
        visa_row.addWidget(self.visa_combo, 1)
        visa_row.addWidget(visa_refresh)
        layout.addRow("K2400 VISA", visa_row)
        self.k_connect_btn = QPushButton("连接 K2400")
        self.k_connect_btn.clicked.connect(self._toggle_k2400)
        layout.addRow("仪器", self.k_connect_btn)
        return group

    def _build_motor_group(self) -> QWidget:
        group = QGroupBox("电机位置模式（手动控制）")
        layout = QFormLayout(group)
        self.motor_direction = self.combo([("CW", 0), ("CCW", 1)])
        self.motor_speed = self.spin(1, 65535, 3000, " RPM")
        self.motor_accel = self.spin(0, 255, 10)
        # QSpinBox is limited to a signed 32-bit integer on Qt.  The X42
        # protocol uses an unsigned 32-bit pulse field, so use a double spin
        # box with zero decimals to expose the complete 0..4294967295 range.
        self.motor_pulses = self.dspin(0, 0xFFFFFFFF, 0, 0, " 脉冲")
        self.motor_mode = self.combo([("相对运动", True), ("绝对运动", False)])
        self.motor_sync = self.combo([("False", False), ("True", True)])
        layout.addRow("方向", self.motor_direction)
        layout.addRow("速度", self.motor_speed)
        layout.addRow("加速度", self.motor_accel)
        layout.addRow("脉冲数", self.motor_pulses)
        layout.addRow("相对/绝对", self.motor_mode)
        layout.addRow("多机同步", self.motor_sync)
        self.motor_send_btn = QPushButton("发送指令")
        self.motor_send_btn.clicked.connect(self._send_motor)
        layout.addRow("", self.motor_send_btn)
        return group

    def _build_k2400_group(self) -> QWidget:
        group = QGroupBox("Keithley 2400 手动控制")
        layout = QFormLayout(group)
        self.voltage_spin = self.dspin(-210.0, 210.0, 5.0, 4, " V")
        self.range_combo = self.combo(
            [("AUTO", None), ("1 µA", 1e-6), ("10 µA", 1e-5), ("100 µA", 1e-4), ("1 mA", 1e-3),
             ("10 mA", 1e-2), ("100 mA", 1e-1), ("1 A", 1.0)]
        )
        layout.addRow("电压", self.voltage_spin)
        layout.addRow("电流量程", self.range_combo)
        self.current_limit_spin = self.dspin(0.001, 1050, 0.1, 4, " mA")
        self.current_limit_spin.setToolTip("输出电流上限，与测量量程不同；连接时读取仪器现有值。高测量量程可能受此值所在档位限制。")
        layout.addRow("电流限值 (Compliance)", self.current_limit_spin)
        apply_row = QHBoxLayout()
        self.apply_k_btn = QPushButton("应用电压/量程")
        self.apply_k_btn.clicked.connect(self._apply_k_settings)
        self.auto_range_btn = QPushButton("AUTO 量程")
        self.auto_range_btn.clicked.connect(self._set_auto_range)
        apply_row.addWidget(self.apply_k_btn)
        apply_row.addWidget(self.auto_range_btn)
        layout.addRow("", apply_row)
        output_row = QHBoxLayout()
        self.output_on_btn = QPushButton("输出 ON")
        self.output_on_btn.clicked.connect(lambda: self._set_output(True))
        self.output_off_btn = QPushButton("输出 OFF")
        self.output_off_btn.clicked.connect(lambda: self._set_output(False))
        output_row.addWidget(self.output_on_btn)
        output_row.addWidget(self.output_off_btn)
        layout.addRow("输出", output_row)
        self.current_label = QLabel("—")
        self.current_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #1565c0;")
        layout.addRow("当前电流", self.current_label)
        read_row = QHBoxLayout()
        self.read_once_btn = QPushButton("读取一次")
        self.read_once_btn.clicked.connect(self._poll_current)
        self.read_start_btn = QPushButton("开始读取")
        self.read_start_btn.clicked.connect(self._start_reading)
        self.read_stop_btn = QPushButton("停止读取")
        self.read_stop_btn.clicked.connect(self._stop_reading)
        self.read_stop_btn.setEnabled(False)
        for button in (self.read_once_btn, self.read_start_btn, self.read_stop_btn):
            read_row.addWidget(button)
        layout.addRow("电流采集", read_row)
        self.read_interval = self.dspin(0.1, 60, 0.5, 2, " s")
        layout.addRow("读取间隔", self.read_interval)
        hint = QLabel("连接后默认不读取。读取一次只采集一个值。\n开始读取后按间隔采集；停止读取保留最后读数。")
        hint.setWordWrap(True)
        layout.addRow(hint)
        return group

    def _build_etch_group(self) -> QWidget:
        group = QGroupBox("自动刻蚀参数")
        layout = QFormLayout(group)
        self.etch_widgets: dict[str, QWidget] = {}
        fields = [
            ("voltage", "刻蚀电压", self.dspin(0, 210, 5.0, 4, " V")),
            ("pulses_per_mm", "脉冲/mm", self.dspin(1, 100000, PULSES_PER_MM, 0)),
            ("target_depth_mm", "下探深度", self.dspin(0, 100, 1.0, 4, " mm")),
            ("immersion_speed", "下探速度", self.spin(1, 65535, 1500, " RPM")),
            ("search_step_pulses", "找液面步进", self.spin(1, 100000, 160, " 脉冲")),
            ("search_speed", "找液面速度", self.spin(1, 65535, 1000, " RPM")),
            ("max_search_mm", "最大搜索行程", self.dspin(0.01, 100, 5.0, 4, " mm")),
            ("search_timeout_s", "找液面超时", self.dspin(1, 3600, 60, 1, " s")),
            ("liquid_threshold_ma", "液面绝对阈值", self.dspin(0, 1000, 0.05, 5, " mA")),
            ("liquid_rise_ma", "液面相对增量", self.dspin(0, 1000, 0.0002, 5, " mA")),
            ("baseline_samples", "空气基线点数", self.spin(1, 1000, 10)),
            ("liquid_confirm_samples", "液面确认点数", self.spin(1, 100, 3)),
            ("search_poll_s", "找液面采样间隔", self.dspin(0.02, 10, 0.30, 2, " s")),
            ("search_move_interval_s", "移动间隔", self.dspin(0.02, 10, 0.35, 2, " s")),
            ("settle_s", "下探稳定时间", self.dspin(0, 600, 2.0, 2, " s")),
            ("sample_interval_s", "刻蚀采样间隔", self.dspin(0.05, 30, 0.30, 2, " s")),
            ("monitor_limit_s", "最大监控时间", self.dspin(0, 86400, 0, 1, " s (0=不限)")),
            ("break_current_ma", "低电流断点阈值", self.dspin(0, 1000, 0.45, 4, " mA")),
            ("break_confirm_samples", "低电流确认点数", self.spin(1, 100, 2)),
            ("overetch_current_ma", "过刻保护阈值", self.dspin(0, 1000, 0.10, 4, " mA")),
            ("min_monitor_s", "最短监控时间", self.dspin(0, 3600, 2.0, 2, " s")),
            ("sudden_drop_ma", "突降绝对值", self.dspin(0, 1000, 0.20, 4, " mA")),
            ("sudden_drop_ratio", "突降相对比例", self.dspin(0, 1, 0.25, 3)),
            ("min_pre_drop_ma", "突降前最小电流", self.dspin(0, 1000, 0.60, 4, " mA")),
            ("max_drop_result_ma", "突降后最大电流", self.dspin(0, 1000, 0.60, 4, " mA")),
            ("drop_baseline_samples", "突降基线点数", self.spin(1, 20, 3)),
            ("drop_confirm_samples", "突降确认点数", self.spin(1, 20, 2)),
            ("drop_tolerance", "突降平台容差", self.dspin(0, 1, 0.10, 3)),
            ("retract_mm", "断点后回撤", self.dspin(0, 100, 2.5, 4, " mm")),
            ("retract_speed", "回撤速度", self.spin(1, 65535, 3000, " RPM")),
            ("acceleration", "电机加速度", self.spin(0, 255, 10)),
            ("max_query_errors", "最大连续通信错误", self.spin(1, 20, 1)),
        ]
        for key, label, widget in fields:
            self.etch_widgets[key] = widget
            layout.addRow(label, widget)
        buttons = QHBoxLayout()
        self.start_btn = QPushButton("▶ Start Etching")
        self.start_btn.setMinimumHeight(36)
        self.start_btn.clicked.connect(self._start_etching)
        self.stop_btn = QPushButton("■ 停止")
        self.stop_btn.setMinimumHeight(36)
        self.stop_btn.clicked.connect(self._stop_etching)
        self.stop_btn.setEnabled(False)
        buttons.addWidget(self.start_btn)
        buttons.addWidget(self.stop_btn)
        layout.addRow("", buttons)
        return group

    def _build_save_group(self) -> QWidget:
        group = QGroupBox("保存文件")
        layout = QFormLayout(group)
        self.save_dir = QLineEdit(str(DEFAULT_SAVE_DIR))
        browse = QPushButton("选择…")
        browse.clicked.connect(self._choose_save_dir)
        row = QHBoxLayout()
        row.addWidget(self.save_dir, 1)
        row.addWidget(browse)
        layout.addRow("保存文件夹", row)
        checks = QHBoxLayout()
        self.save_csv = QCheckBox("CSV")
        self.save_csv.setChecked(True)
        self.save_json = QCheckBox("参数 JSON")
        self.save_json.setChecked(True)
        self.save_png = QCheckBox("曲线 PNG")
        self.save_png.setChecked(True)
        checks.addWidget(self.save_csv)
        checks.addWidget(self.save_json)
        checks.addWidget(self.save_png)
        layout.addRow("保存内容", checks)
        return group

    # ---- connection and manual operations -----------------------------------
    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText() if hasattr(self, "port_combo") else ""
        self.port_combo.clear()
        ports = [] if list_ports is None else [item.device for item in list_ports.comports()]
        if not ports:
            ports = ["COM5"]
        self.port_combo.addItems(ports)
        if current:
            self.port_combo.setCurrentText(current)

    def _refresh_resources(self) -> None:
        if not hasattr(self, "visa_combo"):
            return
        current = self.visa_combo.currentText()
        self.visa_combo.clear()
        resources = self.k2400.resources()
        if not resources:
            resources = ["GPIB0::24::INSTR"]
        self.visa_combo.addItems(resources)
        if current:
            self.visa_combo.setCurrentText(current)

    def _toggle_motor(self) -> None:
        try:
            if self.motor.connected:
                self.motor.close()
                self.motor_connect_btn.setText("打开串口")
                self._log("电机串口已关闭")
            else:
                self.motor.connect(self.port_combo.currentText(), int(self.baud_combo.currentText()))
                self.motor_connect_btn.setText("关闭串口")
                self._log(f"电机已连接：{self.port_combo.currentText()}")
        except Exception as exc:
            self._show_error("电机连接失败", exc)

    def _toggle_k2400(self) -> None:
        try:
            if self.k2400.connected:
                self._stop_reading()
                self.k2400.close()
                self.k_connect_btn.setText("连接 K2400")
                self.current_label.setText("—")
                self._log("K2400 已断开")
            else:
                self.k2400.connect(self.visa_combo.currentText())
                self.current_limit_spin.setValue(float(self.k2400.query(":SENS:CURR:PROT?")) * 1000)
                self.k_connect_btn.setText("断开 K2400")
                self.current_label.setText("未读取")
                self._log(f"K2400 身份验证通过：{self.k2400.identity}；输出 OFF，未开始读取")
        except Exception as exc:
            self.k2400.close()
            self.k_connect_btn.setText("连接 K2400")
            self._show_error("K2400 连接失败", exc)

    def _send_motor(self) -> None:
        try:
            self.motor.send_position(
                int(self.motor_pulses.value()),
                direction=int(self.motor_direction.currentData()),
                speed=self.motor_speed.value(),
                acceleration=self.motor_accel.value(),
                relative=bool(self.motor_mode.currentData()),
                synchronize=bool(self.motor_sync.currentData()),
            )
            self._log(f"发送电机指令：{self.motor_pulses.value()} 脉冲")
        except Exception as exc:
            self._show_error("发送电机指令失败", exc)

    def _apply_k_settings(self) -> None:
        try:
            self.k2400.configure(
                self.voltage_spin.value(), self.range_combo.currentData(),
                self.current_limit_spin.value() / 1000.0,
            )
            self._log("K2400 电压、电流限值和量程已应用并回读确认")
        except Exception as exc:
            self._show_error("应用 K2400 设置失败", exc)

    def _set_auto_range(self) -> None:
        try:
            self.k2400.set_range(None)
            self.range_combo.setCurrentIndex(0)
            self._log("K2400 已切换到 AUTO 量程")
        except Exception as exc:
            self._show_error("设置 AUTO 量程失败", exc)

    def _set_output(self, enabled: bool) -> None:
        try:
            if not enabled:
                self._stop_reading()
            self.k2400.output(enabled)
            self._log("K2400 输出回读确认 " + ("ON" if enabled else "OFF"))
        except Exception as exc:
            self._show_error("切换 K2400 输出失败", exc)

    def _start_reading(self) -> None:
        if not self.k2400.connected:
            self._log("请先连接 K2400，再开始读取")
            return
        if self.worker is not None:
            return
        self.poll_timer.start(round(self.read_interval.value() * 1000))
        self.read_start_btn.setEnabled(False)
        self.read_stop_btn.setEnabled(True)
        self.read_once_btn.setEnabled(False)
        self.read_interval.setEnabled(False)
        self._log(f"开始读取电流，间隔 {self.read_interval.value():g} s")
        self._poll_current()

    def _stop_reading(self) -> None:
        was_active = self.poll_timer.isActive()
        self.poll_timer.stop()
        manual = self.worker is None
        self.read_start_btn.setEnabled(manual)
        self.read_stop_btn.setEnabled(False)
        self.read_once_btn.setEnabled(manual)
        self.read_interval.setEnabled(manual)
        if was_active:
            self._log("已停止手动读取电流（保留最后读数）")

    def _poll_current(self) -> None:
        if self.worker is not None:
            return
        try:
            value_ma = self.k2400.current_a() * 1000.0
            self.current_label.setText(f"{value_ma:.6f} mA")
        except Exception as exc:
            self._stop_reading()
            self.current_label.setText("读取失败")
            self._log(f"电流读取失败，采集已停止：{type(exc).__name__}: {exc}；若为超时，请断开后重新连接 K2400")

    # ---- etching --------------------------------------------------------------
    def _params(self) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, widget in self.etch_widgets.items():
            if isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                result[key] = widget.value()
        # K2400 controls are shared with manual operation and automatic run.
        result["range_value"] = self.range_combo.currentData()
        result["current_limit_a"] = self.current_limit_spin.value() / 1000.0
        return result

    def _start_etching(self) -> None:
        if self.worker is not None:
            return
        if not self.motor.connected or not self.k2400.connected:
            QMessageBox.warning(self, "缺少设备", "请先连接电机串口和 K2400。")
            return
        params = self._params()
        save_dir = Path(self.save_dir.text().strip() or DEFAULT_SAVE_DIR)
        stamp = now_stamp()
        paths: dict[str, Optional[str]] = {
            "csv": str(save_dir / f"etch_{stamp}.csv") if self.save_csv.isChecked() else None,
            "json": str(save_dir / f"etch_{stamp}_parameters.json") if self.save_json.isChecked() else None,
            "png": str(save_dir / f"etch_{stamp}.png") if self.save_png.isChecked() else None,
        }
        self.plot_times.clear()
        self.plot_currents.clear()
        if FigureCanvas is not None:
            self.line.set_data([], [])
            self.axes.relim()
            self.axes.autoscale_view()
            self.canvas.draw_idle()
        self._set_running(True)
        self._log("=" * 55)
        self._log("准备启动自动刻蚀…")
        self.thread = QThread(self)
        self.worker = EtchWorker(self.motor, self.k2400, params, paths)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.log.connect(self._log)
        self.worker.sample.connect(self._on_sample)
        self.worker.finished.connect(self._on_etch_finished)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self._thread_done)
        self.thread.finished.connect(self.thread.deleteLater)
        self.thread.start()

    def _stop_etching(self) -> None:
        if self.worker is not None:
            self._log("正在请求停止，等待当前仪器查询结束…")
            self.worker.request_stop()

    def _on_sample(self, elapsed: float, current_ma: float, phase: str) -> None:
        self.plot_times.append(elapsed)
        self.plot_currents.append(current_ma)
        self.current_label.setText(f"{current_ma:.6f} mA")
        if FigureCanvas is not None and (len(self.plot_times) % 3 == 0 or phase != "monitor"):
            self.line.set_data(self.plot_times, self.plot_currents)
            self.axes.relim()
            self.axes.autoscale_view()
            self.canvas.draw_idle()

    def _on_etch_finished(self, payload: dict[str, Any]) -> None:
        result = payload["result"]
        if FigureCanvas is not None and self.plot_times:
            self.line.set_data(self.plot_times, self.plot_currents)
            self.axes.relim()
            self.axes.autoscale_view()
            self.canvas.draw_idle()
        png_path = result.get("files", {}).get("png")
        if png_path and self.plot_times and FigureCanvas is not None:
            try:
                Path(png_path).parent.mkdir(parents=True, exist_ok=True)
                self.figure.savefig(png_path, dpi=160)
                self._log(f"曲线已保存：{png_path}")
            except Exception as exc:
                self._log(f"保存曲线失败：{exc}")
        csv_path = result.get("files", {}).get("csv")
        json_path = result.get("files", {}).get("json")
        for label, path in (("CSV", csv_path), ("参数 JSON", json_path)):
            if path:
                self._log(f"{label} 已保存：{path}")
        self._log("自动刻蚀结束：" + (result.get("reason") or "完成"))

    def _thread_done(self) -> None:
        self.worker = None
        self.thread = None
        self._set_running(False)
        self.statusBar().showMessage("自动刻蚀线程已结束")

    def _set_running(self, running: bool) -> None:
        self.start_btn.setEnabled(not running)
        self.stop_btn.setEnabled(running)
        for widget in (
            self.motor_send_btn,
            self.motor_connect_btn,
            self.k_connect_btn,
            self.apply_k_btn,
            self.auto_range_btn,
            self.output_on_btn,
            self.output_off_btn,
            self.read_once_btn,
            self.read_start_btn,
            self.read_interval,
        ):
            widget.setEnabled(not running)
        if running:
            self._stop_reading()
            self.read_once_btn.setEnabled(False)
            self.read_start_btn.setEnabled(False)
            self.read_interval.setEnabled(False)

    # ---- misc -----------------------------------------------------------------
    def _choose_save_dir(self) -> None:
        directory = QFileDialog.getExistingDirectory(self, "选择保存文件夹", self.save_dir.text())
        if directory:
            self.save_dir.setText(directory)

    def _log(self, message: str) -> None:
        if hasattr(self, "log_text"):
            self.log_text.append(f"[{datetime.now().strftime('%H:%M:%S')}] {message}")

    def _show_error(self, title: str, exc: Exception) -> None:
        self._log(f"{title}：{exc}")
        QMessageBox.critical(self, title, str(exc))

    def closeEvent(self, event: Any) -> None:  # noqa: N802 - Qt API
        self._stop_reading()
        if self.worker is not None:
            self.worker.request_stop()
            if self.thread is not None:
                self.thread.quit()
                if not self.thread.wait(500):
                    self._log("正在等待自动刻蚀停止并关闭输出，请稍后再关闭窗口")
                    event.ignore()
                    return
        try:
            self.k2400.close()
        finally:
            self.motor.close()
        event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
