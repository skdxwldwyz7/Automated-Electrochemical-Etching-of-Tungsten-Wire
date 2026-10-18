"""Hardware-free regression tests; run: python -m unittest test_k2400_gui -v."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import unittest
from unittest.mock import Mock, patch

import etching_gui as gui


class Instrument:
    def __init__(self):
        self.commands = []
        self.enabled = False
        self.auto = True
        self.range = 0.01
        self.voltage = 0.0
        self.limit = 0.0001
        self.errors = []
        self.identity = "KEITHLEY INSTRUMENTS INC.,MODEL 2400,1234567,C32"
        self.read_error = None
        self.closed = False

    def control_ren(self, mode):
        self.ren = mode

    def close(self):
        self.closed = True

    def write(self, command):
        self.commands.append(command)
        if command.startswith(":OUTP "):
            self.enabled = command.endswith("ON")
        elif command.startswith(":SENS:CURR:RANG:AUTO "):
            self.auto = command.endswith("ON")
        elif command.startswith(":SENS:CURR:RANG "):
            self.range = float(command.split()[-1])
        elif command.startswith(":SOUR:VOLT "):
            self.voltage = float(command.split()[-1])
        elif command.startswith(":SENS:CURR:PROT "):
            self.limit = float(command.split()[-1])

    def query(self, command):
        self.commands.append(command)
        if command == "*IDN?":
            return self.identity
        if command == ":SYST:ERR?":
            return self.errors.pop(0) if self.errors else '0,"No error"'
        if command == ":OUTP?":
            return str(int(self.enabled))
        if command == ":SENS:CURR:RANG:AUTO?":
            return str(int(self.auto))
        if command == ":SENS:CURR:RANG?":
            return str(self.range)
        if command == ":SOUR:VOLT?":
            return str(self.voltage)
        if command == ":SENS:CURR:PROT?":
            return str(self.limit)
        if command == ":READ?":
            if self.read_error:
                raise self.read_error
            return "1.234e-3"
        # Catch any accidental reintroduction of MEAS? or unknown query.
        raise AssertionError(command)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.instrument = Instrument()
        self.manager = Mock()
        self.manager.open_resource.return_value = self.instrument
        self.factory = patch.object(gui.pyvisa, "ResourceManager", return_value=self.manager)
        self.factory.start()
        self.addCleanup(self.factory.stop)
        self.controller = gui.K2400Controller()
        self.addCleanup(self.controller.close)
        self.controller.connect("GPIB0::24::INSTR")

    def test_connect_validates_identity_and_does_not_measure(self):
        self.assertIn("MODEL 2400", self.controller.identity)
        self.assertFalse(self.instrument.enabled)
        self.assertNotIn(":READ?", self.instrument.commands)
        self.assertEqual(self.instrument.ren, gui.pyvisa.constants.RENLineOperation.asrt_address)

    def test_read_preserves_manual_range_voltage_and_output(self):
        self.controller.configure(5.0, 0.01)
        self.controller.output(True)
        self.assertAlmostEqual(self.controller.current_a(), 0.001234)
        self.assertFalse(self.instrument.auto)
        self.assertEqual(self.instrument.range, 0.01)
        self.assertEqual(self.instrument.voltage, 5.0)
        self.assertTrue(self.instrument.enabled)
        self.controller.set_range(None)
        self.assertTrue(self.instrument.auto)

    def test_output_off_is_not_undone_by_read(self):
        self.controller.output(False)
        with self.assertRaisesRegex(RuntimeError, "OFF"):
            self.controller.current_a()
        self.assertFalse(self.instrument.enabled)
        self.assertNotIn(":READ?", self.instrument.commands)

    def test_instrument_errors_are_reported_with_code(self):
        self.instrument.errors = ['-222,"Data out of range"']
        with self.assertRaisesRegex(RuntimeError, "-222"):
            self.controller.set_range(0.01)

    def test_timeout_aborts_and_turns_output_off(self):
        self.controller.output(True)
        self.instrument.read_error = TimeoutError("VISA timeout")
        with self.assertRaises(TimeoutError):
            self.controller.current_a()
        self.assertEqual(self.instrument.commands[-2:], [":ABOR", ":OUTP OFF"])
        self.assertFalse(self.instrument.enabled)

    def test_wrong_identity_cleans_up_resources(self):
        self.instrument.identity = "ACME,OTHER,123,1.0"
        with self.assertRaisesRegex(RuntimeError, "不是 Keithley"):
            self.controller.connect("GPIB0::24::INSTR")
        self.assertFalse(self.controller.connected)
        self.assertTrue(self.instrument.closed)
        self.manager.close.assert_called()

    def test_range_readback_detects_ignored_write(self):
        original = self.instrument.write
        def ignore_range(command):
            if command.startswith(":SENS:CURR:RANG "):
                return
            original(command)
        self.instrument.write = ignore_range
        with self.assertRaisesRegex(RuntimeError, "回读不一致"):
            self.controller.set_range(0.1)

    def test_range_readback_accepts_105_percent_full_scale(self):
        original = self.instrument.query
        def full_scale(command):
            if command == ":SENS:CURR:RANG?":
                return str(self.instrument.range * 1.05)
            return original(command)
        self.instrument.query = full_scale
        self.controller.set_range(0.01)

    def test_compliance_is_applied_before_range(self):
        self.controller.configure(5.0, 0.01, 0.02)
        self.assertEqual(self.instrument.limit, 0.02)
        commands = self.instrument.commands
        self.assertLess(commands.index(":SENS:CURR:PROT 0.02"), commands.index(":SENS:CURR:RANG 0.01"))


class GuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = gui.QApplication.instance() or gui.QApplication([])

    def test_reading_is_opt_in_and_one_shot_is_one_sample(self):
        with patch.object(gui.MainWindow, "_refresh_ports"), patch.object(gui.MainWindow, "_refresh_resources"):
            window = gui.MainWindow()
        controller = Mock()
        controller.connected = True
        controller.identity = "MODEL 2400"
        controller.current_a.return_value = 0.001
        window.k2400 = controller
        try:
            self.assertFalse(window.poll_timer.isActive())
            window.read_once_btn.click()
            self.assertEqual(controller.current_a.call_count, 1)
            self.assertFalse(window.poll_timer.isActive())
            window.read_start_btn.click()
            self.assertTrue(window.poll_timer.isActive())
            window.read_stop_btn.click()
            self.assertFalse(window.poll_timer.isActive())
            self.assertIn("1.000000", window.current_label.text())
            window.read_start_btn.click()
            window._set_output(False)
            self.assertFalse(window.poll_timer.isActive())
            controller.current_a.side_effect = TimeoutError("test timeout")
            window.read_start_btn.click()
            self.assertFalse(window.poll_timer.isActive())
            self.assertIn("test timeout", window.log_text.toPlainText())
        finally:
            window.close()

    def test_invalid_reading_sentinels(self):
        for raw in ("9.91e37", "NaN", "inf", ""):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                gui.parse_current(raw)

    def test_connect_does_not_start_polling_and_loads_existing_limit(self):
        instrument = Instrument()
        manager = Mock()
        manager.open_resource.return_value = instrument
        with patch.object(gui.MainWindow, "_refresh_ports"), patch.object(gui.MainWindow, "_refresh_resources"), patch.object(gui.pyvisa, "ResourceManager", return_value=manager):
            window = gui.MainWindow()
            try:
                window.visa_combo.setCurrentText("GPIB0::24::INSTR")
                window._toggle_k2400()
                self.assertTrue(window.k2400.connected)
                self.assertFalse(window.poll_timer.isActive())
                self.assertNotIn(":READ?", instrument.commands)
                self.assertAlmostEqual(window.current_limit_spin.value(), 0.1)
            finally:
                window.close()


if __name__ == "__main__":
    unittest.main()
