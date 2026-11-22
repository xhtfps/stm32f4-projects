"""Windows desktop launcher for selecting the F407 serial port."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Any

from serial.tools import list_ports


DEFAULT_BAUDRATE = 230400


@dataclass(frozen=True, slots=True)
class SerialPortOption:
    """A serial device as presented in the launcher."""

    device: str
    description: str

    @property
    def label(self) -> str:
        return f"{self.device} - {self.description}" if self.description else self.device


def list_serial_ports(port_lister: Iterable[Any] | None = None) -> list[SerialPortOption]:
    """Return serial ports in a stable order, suitable for a combobox."""
    ports = list_ports.comports() if port_lister is None else port_lister
    options = [
        SerialPortOption(
            device=str(item.device),
            description=str(getattr(item, "description", "") or ""),
        )
        for item in ports
        if str(getattr(item, "device", "")).strip()
    ]
    return sorted(options, key=lambda option: option.device.upper())


class SerialPortLauncher:
    """Small setup window displayed before the OpenCV runtime starts."""

    def __init__(
        self,
        root: tk.Tk,
        *,
        initial_port: str = "",
        initial_baudrate: int = DEFAULT_BAUDRATE,
    ) -> None:
        self.root = root
        self.selected_port: str | None = None
        self.selected_baudrate: int | None = None
        self._options: list[SerialPortOption] = []
        self._port_labels: dict[str, str] = {}
        self.port_value = tk.StringVar(value=initial_port)
        self.baudrate_value = tk.StringVar(value=str(initial_baudrate))
        self.status_value = tk.StringVar()

        root.title("PC-F407 视觉闭环启动")
        root.resizable(False, False)
        root.protocol("WM_DELETE_WINDOW", self._cancel)
        self._build()
        self.refresh_ports(preferred=initial_port)

    def _build(self) -> None:
        frame = ttk.Frame(self.root, padding=20)
        frame.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)

        ttk.Label(
            frame, text="PC-F407 视觉闭环", font=("Microsoft YaHei UI", 15, "bold")
        ).grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(frame, text="选择连接 F407 的串口后启动摄像头程序。").grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(4, 16)
        )

        ttk.Label(frame, text="串口").grid(row=2, column=0, sticky="w", pady=4)
        self.port_combo = ttk.Combobox(
            frame, textvariable=self.port_value, width=36, state="normal"
        )
        self.port_combo.grid(row=2, column=1, sticky="ew", pady=4)

        ttk.Label(frame, text="波特率").grid(row=3, column=0, sticky="w", pady=4)
        self.baudrate_combo = ttk.Combobox(
            frame,
            textvariable=self.baudrate_value,
            values=("230400", "115200", "460800", "921600"),
            width=12,
            state="readonly",
        )
        self.baudrate_combo.grid(row=3, column=1, sticky="w", pady=4)

        ttk.Label(frame, textvariable=self.status_value, foreground="#555555").grid(
            row=4, column=0, columnspan=2, sticky="w", pady=(10, 12)
        )
        buttons = ttk.Frame(frame)
        buttons.grid(row=5, column=0, columnspan=2, sticky="e")
        ttk.Button(buttons, text="刷新串口", command=self.refresh_ports).grid(
            row=0, column=0, padx=(0, 8)
        )
        ttk.Button(buttons, text="取消", command=self._cancel).grid(
            row=0, column=1, padx=(0, 8)
        )
        ttk.Button(buttons, text="启动视觉闭环", command=self._start).grid(
            row=0, column=2
        )

    def refresh_ports(self, preferred: str | None = None) -> None:
        current = preferred if preferred is not None else self._selected_device()
        self._options = list_serial_ports()
        self._port_labels = {option.label: option.device for option in self._options}
        labels = [option.label for option in self._options]
        self.port_combo["values"] = labels
        if current:
            matching = next(
                (option.label for option in self._options if option.device.upper() == current.upper()),
                current,
            )
            self.port_value.set(matching)
        elif labels:
            self.port_value.set(labels[0])
        self.status_value.set(
            f"已检测到 {len(labels)} 个串口" if labels else "未检测到串口，可手动输入，例如 COM5"
        )

    def _selected_device(self) -> str:
        selected = self.port_value.get().strip()
        return self._port_labels.get(selected, selected.split(" - ", 1)[0].strip())

    def _start(self) -> None:
        port = self._selected_device()
        if not port:
            messagebox.showwarning("请选择串口", "请选择或输入连接 F407 的串口，例如 COM5。", parent=self.root)
            return
        try:
            baudrate = int(self.baudrate_value.get())
            if baudrate <= 0:
                raise ValueError
        except ValueError:
            messagebox.showwarning("波特率无效", "请输入正整数波特率。", parent=self.root)
            return
        self.selected_port = port
        self.selected_baudrate = baudrate
        self.root.destroy()

    def _cancel(self) -> None:
        self.root.destroy()


def choose_serial_port(
    *, initial_port: str = "", initial_baudrate: int = DEFAULT_BAUDRATE
) -> tuple[str, int] | None:
    """Show the launcher and return the requested serial settings, or ``None``."""
    root = tk.Tk()
    launcher = SerialPortLauncher(
        root, initial_port=initial_port, initial_baudrate=initial_baudrate
    )
    root.mainloop()
    if launcher.selected_port is None or launcher.selected_baudrate is None:
        return None
    return launcher.selected_port, launcher.selected_baudrate


def launch(config_path: str | Path) -> int:
    """Select serial settings, then invoke the existing real-time controller."""
    from .fast_runtime import RuntimeConfig, run

    settings = RuntimeConfig.load(config_path)
    initial_port = "" if settings.serial_port.lower() == "auto" else settings.serial_port
    selected = choose_serial_port(
        initial_port=initial_port, initial_baudrate=settings.serial_baudrate
    )
    if selected is None:
        return 0
    port, baudrate = selected
    return run(config_path, serial_port=port, serial_baudrate=baudrate)
