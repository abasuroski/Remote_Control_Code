"""
Multi-Motor Control GUI (3 motors)
Sends commands over serial to the STM32 UART command parser.

Protocol (motor index prefix):
  <n>P<val>\n  — position (rad)       motor n=1,2,3
  <n>V<val>\n  — velocity (rad/s)
  <n>K<val>\n  — Kp
  <n>D<val>\n  — Kd
  <n>T<val>\n  — torque feedforward (N·m)
  <n>E\n       — enable motor (AK70/80 only)
  <n>I<val>\n  — set CAN ID
  <n>A<val>\n  — set motor type (60 or 70)

Feedback from STM32 is prefixed: [M<n>] ...

Usage:
  pip install pyserial
  python motor_gui.py
"""

import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading


AK60_PARAMS = {
    "Position (rad)": ("P", -12.56, 12.56, 0.0),
    "Velocity (rad/s)": ("V", -60.0, 60.0, 0.0),
    "Kp": ("K", 0.0, 500.0, 2.0),
    "Kd": ("D", 0.0, 5.0, 1.0),
    "Torque FF (N·m)": ("T", -12.0, 12.0, 0.0),
}

AK70_PARAMS = {
    "Position (rad)": ("P", -12.5, 12.5, 0.0),
    "Velocity (rad/s)": ("V", -30.0, 30.0, 0.0),
    "Kp": ("K", 0.0, 500.0, 6.0),
    "Kd": ("D", 0.0, 5.0, 0.2),
    "Torque FF (N·m)": ("T", -18.0, 18.0, 0.0),
}


class MotorPanel:
    def __init__(self, parent, motor_idx, motor_type, can_id, send_fn):
        self.motor_idx = motor_idx
        self.send_fn = send_fn
        self.sliders = {}
        self.slider_widgets = {}

        self.frame = ttk.LabelFrame(parent, text=f"Motor {motor_idx}")
        self.frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        # --- Type + CAN ID row ---
        config_frame = ttk.Frame(self.frame)
        config_frame.pack(fill="x", padx=5, pady=3)

        ttk.Label(config_frame, text="Type:").pack(side="left")
        self.type_var = tk.StringVar(value=motor_type)
        self.type_combo = ttk.Combobox(config_frame, textvariable=self.type_var,
                                       values=["AK60", "AK70/80"], width=8, state="readonly")
        self.type_combo.pack(side="left", padx=3)
        self.type_combo.bind("<<ComboboxSelected>>", self.on_type_change)

        ttk.Label(config_frame, text="CAN ID:").pack(side="left", padx=(10, 0))
        self.canid_var = tk.IntVar(value=can_id)
        self.canid_entry = ttk.Entry(config_frame, textvariable=self.canid_var, width=5)
        self.canid_entry.pack(side="left", padx=3)

        ttk.Button(config_frame, text="Set ID", command=self.send_canid).pack(side="left", padx=3)

        # --- Enable button (AK70/80 only) ---
        self.enable_frame = ttk.Frame(self.frame)
        self.enable_frame.pack(fill="x", padx=5, pady=2)
        self.enable_btn = ttk.Button(self.enable_frame, text="Enable Motor", command=self.send_enable)
        self.enable_btn.pack(side="left")

        # --- Sliders frame ---
        self.slider_frame = ttk.LabelFrame(self.frame, text="Parameters")
        self.slider_frame.pack(fill="x", padx=5, pady=3)

        self.build_sliders()

        # --- GO button ---
        ttk.Button(self.frame, text="GO", command=self.send_all_params).pack(fill="x", padx=5, pady=5, ipady=6)

        # --- Feedback ---
        fb_frame = ttk.LabelFrame(self.frame, text="Feedback")
        fb_frame.pack(fill="both", expand=True, padx=5, pady=3)

        self.feedback_text = tk.Text(fb_frame, height=6, width=40, state="disabled", font=("Consolas", 8))
        self.feedback_text.pack(fill="both", expand=True, padx=3, pady=3)

        self.update_enable_visibility()

    def build_sliders(self):
        for widget in self.slider_frame.winfo_children():
            widget.destroy()
        self.sliders.clear()
        self.slider_widgets.clear()

        params = AK70_PARAMS if "70" in self.type_var.get() else AK60_PARAMS

        for i, (label, (cmd, lo, hi, default)) in enumerate(params.items()):
            ttk.Label(self.slider_frame, text=label).grid(row=i, column=0, sticky="w", padx=3, pady=1)

            var = tk.DoubleVar(value=default)
            slider = ttk.Scale(self.slider_frame, from_=lo, to=hi, variable=var,
                               orient="horizontal", length=180)
            slider.grid(row=i, column=1, padx=3, pady=1)

            entry = ttk.Entry(self.slider_frame, textvariable=var, width=7)
            entry.grid(row=i, column=2, padx=3, pady=1)

            self.sliders[cmd] = var

    def on_type_change(self, event=None):
        self.build_sliders()
        self.update_enable_visibility()
        self.send_fn(f"{self.motor_idx}A{'70' if '70' in self.type_var.get() else '60'}\n")

    def update_enable_visibility(self):
        if "70" in self.type_var.get():
            self.enable_btn.pack(side="left")
        else:
            self.enable_btn.pack_forget()

    def send_canid(self):
        self.send_fn(f"{self.motor_idx}I{self.canid_var.get()}\n")

    def send_enable(self):
        self.send_fn(f"{self.motor_idx}E\n")

    def send_all_params(self):
        for cmd, var in self.sliders.items():
            self.send_fn(f"{self.motor_idx}{cmd}{var.get():.4f}\n")

    def append_feedback(self, line):
        self.feedback_text.config(state="normal")
        self.feedback_text.insert("end", line + "\n")
        self.feedback_text.see("end")
        if int(self.feedback_text.index("end-1c").split(".")[0]) > 200:
            self.feedback_text.delete("1.0", "2.0")
        self.feedback_text.config(state="disabled")


class MultiMotorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Multi-Motor Control")
        self.ser = None

        # --- Connection frame ---
        conn_frame = ttk.LabelFrame(root, text="Connection")
        conn_frame.pack(fill="x", padx=10, pady=5)

        self.port_var = tk.StringVar()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, values=ports, width=15)
        self.port_combo.pack(side="left", padx=5, pady=5)
        if ports:
            self.port_combo.current(0)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=5)

        self.status_label = ttk.Label(conn_frame, text="Disconnected")
        self.status_label.pack(side="left", padx=10)

        # --- Motor panels (3 side-by-side) ---
        motors_frame = ttk.Frame(root)
        motors_frame.pack(fill="both", expand=True, padx=5, pady=5)

        self.panels = []
        motor_configs = [
            ("AK60", 104),
            ("AK70/80", 1),
            ("AK70/80", 2),
        ]
        for idx, (mtype, cid) in enumerate(motor_configs, start=1):
            panel = MotorPanel(motors_frame, idx, mtype, cid, self.send_cmd)
            self.panels.append(panel)

        # --- Serial reader ---
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def toggle_connect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Disconnected")
        else:
            port = self.port_var.get()
            if not port:
                return
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text=f"Connected: {port}")
            except serial.SerialException as e:
                self.status_label.config(text=f"Error: {e}")

    def send_cmd(self, cmd):
        if self.ser and self.ser.is_open:
            self.ser.write(cmd.encode())

    def read_serial(self):
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    line = self.ser.readline().decode(errors="replace").strip()
                    if line:
                        self.route_feedback(line)
                except Exception:
                    pass

    def route_feedback(self, line):
        # Expected format: [M1] ... or [M2] ... or [M3] ...
        routed = False
        for i, panel in enumerate(self.panels, start=1):
            tag = f"[M{i}]"
            if line.startswith(tag):
                self.root.after(0, panel.append_feedback, line[len(tag):].strip())
                routed = True
                break
        if not routed:
            # Fallback: show on all panels or first panel
            self.root.after(0, self.panels[0].append_feedback, line)

    def on_close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = MultiMotorGUI(root)
    root.mainloop()
