import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re

# Adjust COM port and baud rate
ser = serial.Serial('COM3', 115200, timeout=0.01)  # Important: small timeout
ser.flush()

# Buffer to hold the last N data points
max_len = 100
time_axis = deque(maxlen=max_len)

# Create separate deques for each signal
pendulum_angles = deque(maxlen=max_len)
pendulum_velocities = deque(maxlen=max_len)
motor_velocities = deque(maxlen=max_len)
pwm_duties = deque(maxlen=max_len)
control_inputs = deque(maxlen=max_len)
loop_times = deque(maxlen=max_len)

# Set up the plot
fig, axs = plt.subplots(3, 2, figsize=(12, 8))
axs = axs.flatten()

lines = []
titles = [
    "Pendulum Angle (rad)", 
    "Pendulum Angular Velocity (rad/s)", 
    "Motor Velocity (rad/s)", 
    "PWM Duty Cycle", 
    "Control Input (V)", 
    "Loop Time (s)"
]

data_deques = [
    pendulum_angles,
    pendulum_velocities,
    motor_velocities,
    pwm_duties,
    control_inputs,
    loop_times
]

for ax, title in zip(axs, titles):
    line, = ax.plot([], [], lw=2)
    lines.append(line)
    ax.set_title(title)
    ax.set_xlim(0, max_len)
    ax.grid()

# Precompile regex for faster parsing
pattern = re.compile(r"^\s*(.*?)\s*:\s*(-?\d+\.?\d*)$")

# Buffer for incoming serial lines
serial_buffer = []

def update(frame):
    try:
        # Read all available bytes
        while ser.in_waiting:
            line_raw = ser.readline().decode('utf-8', errors='ignore').strip()
            if line_raw:
                serial_buffer.append(line_raw)

        # Process all buffered lines
        while serial_buffer:
            line = serial_buffer.pop(0)
            print(f"Received: {line}")

            match = pattern.match(line)
            if match:
                label, value = match.groups()
                value = float(value)

                if label.startswith("Pendulum Angle"):
                    pendulum_angles.append(value)
                elif label.startswith("Pendulum Velocity"):
                    pendulum_velocities.append(value)
                elif label.startswith("Motor Velocity"):
                    motor_velocities.append(value)
                elif label.startswith("PWM Duty cycle"):
                    pwm_duties.append(value)
                elif label.startswith("Control Input"):
                    control_inputs.append(value)
                elif label.startswith("Loop Time"):
                    loop_times.append(value)
                    if time_axis:
                        time_axis.append(time_axis[-1] + 1)
                    else:
                        time_axis.append(0)

        # Update plots
        for ax, line, data in zip(axs, lines, data_deques):
            line.set_data(range(len(data)), list(data))
            ax.set_xlim(0, max(len(data), max_len))
            if data:
                ymin = min(data) * 0.9
                ymax = max(data) * 1.1
                if ymin == ymax:
                    ymin -= 1
                    ymax += 1
                ax.set_ylim(ymin, ymax)

    except Exception as e:
        print("Error:", e)

    return lines

# Create animation
ani = animation.FuncAnimation(fig, update, interval=1)  # Faster interval
plt.tight_layout()
plt.show()
