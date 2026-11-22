import unittest
from types import SimpleNamespace

from lissajous_vision.desktop_launcher import SerialPortOption, list_serial_ports


class DesktopLauncherTests(unittest.TestCase):
    def test_port_options_include_description_and_sort_by_device(self):
        options = list_serial_ports([
            SimpleNamespace(device="COM9", description="Debug adapter"),
            SimpleNamespace(device="COM3", description="USB UART"),
        ])
        self.assertEqual(
            [SerialPortOption("COM3", "USB UART"), SerialPortOption("COM9", "Debug adapter")],
            options,
        )
        self.assertEqual("COM3 - USB UART", options[0].label)

    def test_port_options_ignore_missing_device(self):
        options = list_serial_ports([SimpleNamespace(device="", description="Unknown")])
        self.assertEqual([], options)


if __name__ == "__main__":
    unittest.main()
