import unittest
from types import SimpleNamespace

from lissajous_vision.serial_link import SerialLink, SerialLinkError, resolve_port
from lissajous_vision.serial_protocol import AppliedFrame, SetCommand


class FakeSerial:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.rx = bytearray()
        self.tx = bytearray()
        self.is_open = True

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, size):
        result = bytes(self.rx[:size])
        del self.rx[:size]
        return result

    def write(self, data):
        self.tx.extend(data)
        return len(data)

    def close(self):
        self.is_open = False


class SerialLinkTests(unittest.TestCase):
    def test_resolve_explicit_port(self):
        self.assertEqual("COM9", resolve_port("COM9", lambda: []))

    def test_auto_requires_exactly_one_port(self):
        one = lambda: [SimpleNamespace(device="COM4", description="USB UART")]
        self.assertEqual("COM4", resolve_port("auto", one))
        with self.assertRaisesRegex(SerialLinkError, "未检测到"):
            resolve_port("auto", lambda: [])
        many = lambda: [
            SimpleNamespace(device="COM4", description="A"),
            SimpleNamespace(device="COM7", description="B"),
        ]
        with self.assertRaisesRegex(SerialLinkError, "COM4.*COM7"):
            resolve_port("auto", many)

    def test_poll_send_and_close(self):
        created = []

        def factory(**kwargs):
            port = FakeSerial(**kwargs)
            created.append(port)
            return port

        link = SerialLink("COM5", serial_factory=factory, port_lister=lambda: [])
        port = created[0]
        self.assertEqual(230400, port.kwargs["baudrate"])
        self.assertEqual(0, port.kwargs["timeout"])
        self.assertEqual([], link.poll())
        port.rx.extend(b"$APPLIED,2,8\r\n")
        self.assertEqual([AppliedFrame(2, 8)], link.poll())
        link.send_set(SetCommand(2, 8, 10_000_000, 0, 471, 1))
        self.assertEqual(
            b"$SET,2,8,10000000,0,471,1\r\n", bytes(port.tx)
        )
        link.close()
        self.assertFalse(port.is_open)

    def test_short_write_and_closed_port_raise(self):
        class ShortSerial(FakeSerial):
            def write(self, data):
                return max(0, len(data) - 1)

        link = SerialLink(
            "COM2", serial_factory=lambda **kw: ShortSerial(**kw),
            port_lister=lambda: [],
        )
        with self.assertRaisesRegex(SerialLinkError, "写入不完整"):
            link.send_set(SetCommand(1, 1, 1_000_000, 0, 100, 1))
        link.close()
        with self.assertRaisesRegex(SerialLinkError, "已关闭"):
            link.poll()


if __name__ == "__main__":
    unittest.main()
