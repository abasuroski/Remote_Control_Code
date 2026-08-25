"""
Robot Arm Control GUI (Body + End Effector)
Sends commands over serial to the STM32 UART command parser.

Protocol (motor index prefix):
  <n>P<val>\n  — position (rad for motors, degrees for servos)
  <n>V<val>\n  — velocity (rad/s)
  <n>K<val>\n  — Kp
  <n>D<val>\n  — Kd
  <n>T<val>\n  — torque feedforward (N·m)
  <n>E\n       — enable motor (AK70/80/AK40 only)
  <n>I<val>\n  — set CAN ID
  <n>A<val>\n  — set motor type (60, 70, or 40)

  Body motors: n=1 (Base), 2 (Shoulder), 3 (Elbow), 4 (Linkage)
  End effector servos: n=5 (Wrist 1), 6 (Wrist 2), 7 (Gripper)

Servo PWM reference (from servotest project):
  TIM2, Prescaler=180-1, Period=10000-1 → 50Hz
  Pulse 250 = 0 deg (500us), Pulse 1250 = 180 deg (2500us)
  MG995 TowerPro servos, channels TIM2_CH2, TIM2_CH3, TIM2_CH4

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
from datetime import datetime


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

AK40_PARAMS = {
    "Position (rad)": ("P", -12.5, 12.5, 0.0),
    "Velocity (rad/s)": ("V", -45.5, 45.5, 0.0),
    "Kp": ("K", 0.0, 500.0, 6.0),
    "Kd": ("D", 0.0, 5.0, 0.2),
    "Torque FF (N·m)": ("T", -5.0, 5.0, 0.0),
}


class MotorPanel:
    def __init__(self, parent, motor_idx, label, motor_type, can_id, send_fn):
        self.motor_idx = motor_idx
        self.send_fn = send_fn
        self.sliders = {}
        self.slider_widgets = {}

        self.frame = ttk.LabelFrame(parent, text=label)
        self.frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        # --- Type + CAN ID row ---
        config_frame = ttk.Frame(self.frame)
        config_frame.pack(fill="x", padx=5, pady=3)

        ttk.Label(config_frame, text="Type:").pack(side="left")
        self.type_var = tk.StringVar(value=motor_type)
        self.type_combo = ttk.Combobox(config_frame, textvariable=self.type_var,
                                       values=["AK60", "AK70/80", "AK40"], width=8, state="readonly")
        self.type_combo.pack(side="left", padx=3)
        self.type_combo.bind("<<ComboboxSelected>>", self.on_type_change)

        ttk.Label(config_frame, text="CAN ID:").pack(side="left", padx=(10, 0))
        self.canid_var = tk.IntVar(value=can_id)
        self.canid_entry = ttk.Entry(config_frame, textvariable=self.canid_var, width=5)
        self.canid_entry.pack(side="left", padx=3)

        ttk.Button(config_frame, text="Set ID", command=self.send_canid).pack(side="left", padx=3)

        # --- Enable button ---
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

        t = self.type_var.get()
        if "70" in t:
            params = AK70_PARAMS
        elif "40" in t:
            params = AK40_PARAMS
        else:
            params = AK60_PARAMS

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
        t = self.type_var.get()
        if "70" in t:
            code = "70"
        elif "40" in t:
            code = "40"
        else:
            code = "60"
        self.send_fn(f"{self.motor_idx}A{code}\n")

    def update_enable_visibility(self):
        t = self.type_var.get()
        if "70" in t or "40" in t:
            self.enable_btn.pack(side="left")
        else:
            self.enable_btn.pack_forget()

    def send_canid(self):
        self.send_fn(f"{self.motor_idx}I{self.canid_var.get()}\n")

    def send_enable(self):
        self.send_fn(f"{self.motor_idx}E\n")

    def send_all_params(self):
        import time
        for cmd, var in self.sliders.items():
            self.send_fn(f"{self.motor_idx}{cmd}{var.get():.4f}\n")
            time.sleep(0.005)

    def append_feedback(self, line):
        self.feedback_text.config(state="normal")
        self.feedback_text.insert("end", line + "\n")
        self.feedback_text.see("end")
        if int(self.feedback_text.index("end-1c").split(".")[0]) > 200:
            self.feedback_text.delete("1.0", "2.0")
        self.feedback_text.config(state="disabled")


class ServoPanel:
    def __init__(self, parent, servo_idx, label, send_fn):
        self.servo_idx = servo_idx
        self.send_fn = send_fn

        self.frame = ttk.LabelFrame(parent, text=label)
        self.frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        # --- Info ---
        ttk.Label(self.frame, text="MG995 TowerPro", font=("", 8, "italic")).pack(padx=5, pady=2)

        # --- Position slider ---
        pos_frame = ttk.LabelFrame(self.frame, text="Position (degrees)")
        pos_frame.pack(fill="x", padx=5, pady=5)

        self.pos_var = tk.DoubleVar(value=90.0)

        slider = ttk.Scale(pos_frame, from_=0, to=180, variable=self.pos_var,
                           orient="horizontal", length=200)
        slider.pack(padx=5, pady=3)

        entry_frame = ttk.Frame(pos_frame)
        entry_frame.pack(fill="x", padx=5, pady=3)

        self.pos_entry = ttk.Entry(entry_frame, textvariable=self.pos_var, width=7)
        self.pos_entry.pack(side="left", padx=3)
        ttk.Label(entry_frame, text="deg").pack(side="left")

        # --- Preset buttons ---
        preset_frame = ttk.Frame(self.frame)
        preset_frame.pack(fill="x", padx=5, pady=3)
        for angle in [0, 45, 90, 135, 180]:
            ttk.Button(preset_frame, text=f"{angle}°", width=4,
                       command=lambda a=angle: self.set_angle(a)).pack(side="left", padx=2)

        # --- GO button ---
        ttk.Button(self.frame, text="GO", command=self.send_position).pack(fill="x", padx=5, pady=5, ipady=6)

        # --- Feedback ---
        fb_frame = ttk.LabelFrame(self.frame, text="Feedback")
        fb_frame.pack(fill="both", expand=True, padx=5, pady=3)

        self.feedback_text = tk.Text(fb_frame, height=4, width=30, state="disabled", font=("Consolas", 8))
        self.feedback_text.pack(fill="both", expand=True, padx=3, pady=3)

    def set_angle(self, angle):
        self.pos_var.set(angle)

    def send_position(self):
        angle = self.pos_var.get()
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180
        self.send_fn(f"{self.servo_idx}P{angle:.1f}\n")

    def append_feedback(self, line):
        self.feedback_text.config(state="normal")
        self.feedback_text.insert("end", line + "\n")
        self.feedback_text.see("end")
        if int(self.feedback_text.index("end-1c").split(".")[0]) > 100:
            self.feedback_text.delete("1.0", "2.0")
        self.feedback_text.config(state="disabled")


class RobotArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Arm Control")
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

        # --- Recording frame ---
        rec_frame = ttk.LabelFrame(root, text="Recording")
        rec_frame.pack(fill="x", padx=10, pady=5)

        self.record_btn = ttk.Button(rec_frame, text="Record", command=self.start_recording)
        self.record_btn.pack(side="left", padx=5, pady=5)

        self.stop_btn = ttk.Button(rec_frame, text="Stop Record", command=self.stop_recording, state="disabled")
        self.stop_btn.pack(side="left", padx=5, pady=5)

        self.rec_label = ttk.Label(rec_frame, text="")
        self.rec_label.pack(side="left", padx=10)

        self.log_file = None

        # --- Tabbed notebook ---
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True, padx=5, pady=5)

        # === Body Tab ===
        body_frame = ttk.Frame(self.notebook)
        self.notebook.add(body_frame, text="Body")

        self.panels = []
        motor_configs = [
            (1, "Base", "AK60", 104),
            (2, "Shoulder", "AK70/80", 1),
            (3, "Elbow", "AK70/80", 2),
            (4, "Linkage", "AK40", 3),
        ]
        for idx, label, mtype, cid in motor_configs:
            panel = MotorPanel(body_frame, idx, label, mtype, cid, self.send_cmd)
            self.panels.append(panel)

        # === End Effector Tab ===
        ee_frame = ttk.Frame(self.notebook)
        self.notebook.add(ee_frame, text="End Effector")

        self.servo_panels = []
        servo_configs = [
            (5, "Wrist 1"),
            (6, "Wrist 2"),
            (7, "Gripper"),
        ]
        for idx, label in servo_configs:
            sp = ServoPanel(ee_frame, idx, label, self.send_cmd)
            self.servo_panels.append(sp)

        # --- GO ALL servos button ---
        ttk.Button(ee_frame, text="GO ALL", command=self.send_all_servos).pack(fill="x", padx=10, pady=5, ipady=6)

        # --- Serial reader ---
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def send_all_servos(self):
        import time
        for sp in self.servo_panels:
            sp.send_position()
            time.sleep(0.005)

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

    def start_recording(self):
        filename = datetime.now().strftime("log_%Y-%m-%d_%H-%M-%S.txt")
        self.log_file = open(filename, "w")
        self.record_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.rec_label.config(text=f"Recording: {filename}")

    def stop_recording(self):
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        self.record_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.rec_label.config(text="Stopped")

    def route_feedback(self, line):
        if self.log_file:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self.log_file.write(f"{timestamp}  {line}\n")
            self.log_file.flush()

        routed = False
        # Check motor panels [M1]-[M4]
        for i, panel in enumerate(self.panels, start=1):
            tag = f"[M{i}]"
            if line.startswith(tag):
                self.root.after(0, panel.append_feedback, line[len(tag):].strip())
                routed = True
                break
        # Check servo panels [M5]-[M7]
        if not routed:
            for i, sp in enumerate(self.servo_panels, start=5):
                tag = f"[M{i}]"
                if line.startswith(tag):
                    self.root.after(0, sp.append_feedback, line[len(tag):].strip())
                    routed = True
                    break
        if not routed:
            self.root.after(0, self.panels[0].append_feedback, line)

    def on_close(self):
        self.running = False
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmGUI(root)
    root.mainloop()
