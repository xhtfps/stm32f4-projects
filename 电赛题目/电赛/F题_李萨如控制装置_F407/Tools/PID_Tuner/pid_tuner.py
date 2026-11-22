# -*- coding: utf-8 -*-
"""STM32 USART1 (PA9/PA10) runtime PID tuning tool."""

from __future__ import annotations

import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - startup dependency check
    raise SystemExit("缺少 pyserial，请先执行: pip install -r requirements.txt") from exc


LOOPS = (
    ("FREQ", "频率环"),
    ("FREQ_LOW", "低频频率环"),
    ("PHASE", "相位环"),
    ("PHASE_LOW", "低频相位环"),
    ("AMP", "幅值环"),
)
FIELDS = ("Kp", "Ki", "Kd", "Alpha")
DEFAULTS = {
    "FREQ": ("1.00", "2.00", "0.0", "0.0"),
    "FREQ_LOW": ("0.25", "0.03", "0.0", "0.0"),
    "PHASE": ("0.20", "0.14", "0.001", "0.05"),
    "PHASE_LOW": ("0.08", "0.10", "0.001", "0.05"),
    "AMP": ("0.015", "0.030", "0.0", "0.0"),
}


class PidTunerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("李萨如控制装置 PID 调参")
        self.root.geometry("760x520")
        self.root.minsize(700, 470)

        self.serial_port: serial.Serial | None = None
        self.reader_thread: threading.Thread | None = None
        self.reader_stop = threading.Event()
        self.rx_queue: queue.Queue[str] = queue.Queue()
        self.tx_lock = threading.Lock()

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="230400")
        self.status_var = tk.StringVar(value="未连接")
        self.values: dict[str, dict[str, tk.StringVar]] = {}

        self._build_ui()
        self.refresh_ports()
        self.root.after(50, self._drain_rx_queue)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        connection = ttk.Frame(self.root, padding=(12, 12, 12, 6))
        connection.grid(row=0, column=0, sticky="ew")
        connection.columnconfigure(1, weight=1)

        ttk.Label(connection, text="串口").grid(row=0, column=0, padx=(0, 6))
        self.port_box = ttk.Combobox(
            connection, textvariable=self.port_var, state="readonly", width=26
        )
        self.port_box.grid(row=0, column=1, sticky="ew")
        ttk.Button(connection, text="刷新", command=self.refresh_ports).grid(
            row=0, column=2, padx=6
        )
        ttk.Label(connection, text="波特率").grid(row=0, column=3, padx=(10, 6))
        ttk.Entry(connection, textvariable=self.baud_var, width=10).grid(
            row=0, column=4
        )
        self.connect_button = ttk.Button(
            connection, text="连接", command=self.toggle_connection
        )
        self.connect_button.grid(row=0, column=5, padx=(8, 0))

        panel = ttk.LabelFrame(self.root, text="控制参数", padding=12)
        panel.grid(row=1, column=0, padx=12, pady=6, sticky="ew")
        panel.columnconfigure(0, weight=1)
        for column in range(1, 5):
            panel.columnconfigure(column, weight=2)

        ttk.Label(panel, text="控制环").grid(row=0, column=0, sticky="w")
        for column, field in enumerate(FIELDS, start=1):
            ttk.Label(panel, text=field).grid(row=0, column=column, padx=5)
        ttk.Label(panel, text="操作").grid(row=0, column=5, padx=(8, 0))

        for row, (loop_name, display_name) in enumerate(LOOPS, start=1):
            ttk.Label(panel, text=display_name).grid(
                row=row, column=0, sticky="w", pady=6
            )
            loop_values: dict[str, tk.StringVar] = {}
            for column, (field, initial) in enumerate(
                zip(FIELDS, DEFAULTS[loop_name]), start=1
            ):
                value = tk.StringVar(value=initial)
                loop_values[field] = value
                ttk.Entry(panel, textvariable=value, width=14).grid(
                    row=row, column=column, padx=5, sticky="ew"
                )
            self.values[loop_name] = loop_values
            ttk.Button(
                panel,
                text="下发",
                command=lambda name=loop_name: self.apply_loop(name),
            ).grid(row=row, column=5, padx=(8, 0))

        actions = ttk.Frame(panel)
        actions.grid(row=4, column=0, columnspan=6, sticky="e", pady=(10, 0))
        ttk.Button(actions, text="读取参数", command=self.query_parameters).pack(
            side="left", padx=4
        )
        ttk.Button(actions, text="全部下发", command=self.apply_all).pack(
            side="left", padx=4
        )

        log_frame = ttk.LabelFrame(self.root, text="通信记录", padding=8)
        log_frame.grid(row=2, column=0, padx=12, pady=(6, 8), sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = tk.Text(log_frame, height=10, wrap="none", state="disabled")
        self.log.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=scrollbar.set)

        status = ttk.Label(
            self.root, textvariable=self.status_var, anchor="w", padding=(12, 0, 12, 10)
        )
        status.grid(row=3, column=0, sticky="ew")

    def refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        self.port_box["values"] = ports
        if self.port_var.get() not in ports:
            self.port_var.set(ports[0] if ports else "")

    def toggle_connection(self) -> None:
        if self.serial_port is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("连接失败", "未找到可用串口")
            return
        try:
            baudrate = int(self.baud_var.get())
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.5,
            )
        except (ValueError, serial.SerialException) as exc:
            self.serial_port = None
            messagebox.showerror("连接失败", str(exc))
            return

        self.reader_stop.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.connect_button.configure(text="断开")
        self.status_var.set(f"已连接 {port} @ {baudrate}")
        self._append_log(f"连接 {port} @ {baudrate}")
        self.root.after(150, self.query_parameters)

    def disconnect(self) -> None:
        self.reader_stop.set()
        port = self.serial_port
        self.serial_port = None
        if port is not None:
            try:
                port.close()
            except serial.SerialException:
                pass
        self.connect_button.configure(text="连接")
        self.status_var.set("未连接")
        self._append_log("串口已断开")

    def _reader_loop(self) -> None:
        while not self.reader_stop.is_set():
            port = self.serial_port
            if port is None:
                return
            try:
                raw_line = port.readline()
            except serial.SerialException as exc:
                self.rx_queue.put(f"@LOCAL_ERROR,{exc}")
                return
            if not raw_line:
                continue
            line = raw_line.decode("ascii", errors="ignore").strip()
            marker = line.find("@")
            if marker >= 0:
                self.rx_queue.put(line[marker:])

    def _drain_rx_queue(self) -> None:
        try:
            while True:
                self._handle_response(self.rx_queue.get_nowait())
        except queue.Empty:
            pass
        self.root.after(50, self._drain_rx_queue)

    def _handle_response(self, line: str) -> None:
        self._append_log(line)
        if line.startswith("@PID,"):
            parts = line.split(",")
            if len(parts) == 6 and parts[1] in self.values:
                for field, value in zip(FIELDS, parts[2:]):
                    self.values[parts[1]][field].set(value)
                self.status_var.set(f"已读取 {parts[1]} 参数")
        elif line.startswith("@OK"):
            self.status_var.set("参数已应用")
        elif line.startswith("@ERR"):
            self.status_var.set(f"设备拒绝命令：{line}")
        elif line.startswith("@LOCAL_ERROR"):
            self.status_var.set(line)
            self.disconnect()

    def _send(self, command: str) -> bool:
        port = self.serial_port
        if port is None or not port.is_open:
            messagebox.showwarning("串口未连接", "请先连接设备")
            return False
        try:
            with self.tx_lock:
                port.write((command + "\n").encode("ascii"))
                port.flush()
        except serial.SerialException as exc:
            messagebox.showerror("发送失败", str(exc))
            return False
        self._append_log(f"> {command}")
        return True

    def query_parameters(self) -> None:
        self._send("PID GET")

    def apply_loop(self, loop_name: str) -> bool:
        try:
            numbers = [float(self.values[loop_name][field].get()) for field in FIELDS]
        except ValueError:
            messagebox.showerror("参数错误", f"{loop_name} 包含非数字参数")
            return False
        if any(value < 0.0 or value > 1000.0 for value in numbers[:3]):
            messagebox.showerror("参数错误", "Kp、Ki、Kd 必须在 0~1000 之间")
            return False
        if not 0.0 <= numbers[3] <= 1.0:
            messagebox.showerror("参数错误", "Alpha 必须在 0~1 之间")
            return False

        payload = " ".join(f"{value:.6f}" for value in numbers)
        return self._send(f"PID SET {loop_name} {payload}")

    def apply_all(self) -> None:
        for loop_name, _ in LOOPS:
            if not self.apply_loop(loop_name):
                return

    def _append_log(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def close(self) -> None:
        self.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    style = ttk.Style(root)
    if "vista" in style.theme_names():
        style.theme_use("vista")
    PidTunerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
