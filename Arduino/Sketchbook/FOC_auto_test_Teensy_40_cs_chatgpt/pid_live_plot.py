#!/usr/bin/env python3
"""
Real-time PID analysis plotter for SimpleFOC test logs
Teensy 4.0 + Ninebot wheel
"""

import serial
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time

# ======= USER SETTINGS =======
PORT = "/dev/ttyACM0"    # adjust for your OS (e.g., COM5 on Windows)
BAUD = 115200
MAX_POINTS = 1000          # number of samples to show live
SAVE_LOG = True
LOG_FILE = "pid_log.csv"
# =============================

# open serial port
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2.0)
print(f"Connected to {PORT}")

# initialize storage
data = {"time": deque(maxlen=MAX_POINTS),
        "target": deque(maxlen=MAX_POINTS),
        "velocity": deque(maxlen=MAX_POINTS),
        "iq": deque(maxlen=MAX_POINTS),
        "vq": deque(maxlen=MAX_POINTS)}

if SAVE_LOG:
    with open(LOG_FILE, "w") as f:
        f.write("time_ms,target_vel,meas_vel,iq,voltage_q\n")

# setup matplotlib
plt.style.use("seaborn-v0_8-darkgrid")
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
lines = []
labels = ["Velocity", "Current q", "Voltage q"]
colors = ["tab:blue", "tab:orange", "tab:green"]

for ax, label, color in zip(axs, labels, colors):
    ax.set_ylabel(label)
    line, = ax.plot([], [], color=color)
    lines.append(line)
axs[-1].set_xlabel("Time (s)")

plt.tight_layout()

def update(frame):
    # read and parse serial lines
    while ser.in_waiting:
        try:
            line = ser.readline().decode().strip()
            if not line or line.startswith("time_ms"):  # skip header
                continue
            t_ms, tgt, vel, iq, vq = map(float, line.split(","))
            data["time"].append(t_ms / 1000.0)
            data["target"].append(tgt)
            data["velocity"].append(vel)
            data["iq"].append(iq)
            data["vq"].append(vq)
            if SAVE_LOG:
                with open(LOG_FILE, "a") as f:
                    f.write(line + "\n")
        except Exception:
            pass

    # update plots
    if len(data["time"]) > 5:
        t = list(data["time"])
        lines[0].set_data(t, data["velocity"])
        lines[1].set_data(t, data["iq"])
        lines[2].set_data(t, data["vq"])
        for ax in axs:
            ax.relim()
            ax.autoscale_view()

    return lines

ani = animation.FuncAnimation(fig, update, interval=100, blit=False)

try:
    print("Plotting live... press Ctrl+C to stop")
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("Serial closed.")

