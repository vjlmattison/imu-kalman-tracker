"""
live_plot_2axis.py

Same idea as live_plot.py, but tracks BOTH pitch and roll simultaneously,
each with its own independent Kalman filter instance -- two separate
1D filters running side by side, not one coupled 2D model. That's a
deliberate, defensible scope choice (see project README/plan for why).

Two subplots, stacked: top = pitch, bottom = roll. Each shows the same
three-line comparison (accel-only, gyro-only, Kalman-fused).
"""

import math
import serial
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from kalman import KalmanFilter1D

SERIAL_PORT = "COM5"       # EDIT to match your Arduino's port
BAUD_RATE = 115200
WINDOW_SIZE = 300

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Listening on {SERIAL_PORT}...")

    # Two independent filter instances -- pitch and roll don't share state.
    kf_pitch = KalmanFilter1D(q_angle=0.001, q_bias=0.003, r_measure=0.03)
    kf_roll = KalmanFilter1D(q_angle=0.001, q_bias=0.003, r_measure=0.03)

    times = deque(maxlen=WINDOW_SIZE)

    pitch_accel = deque(maxlen=WINDOW_SIZE)
    pitch_gyro = deque(maxlen=WINDOW_SIZE)
    pitch_kalman = deque(maxlen=WINDOW_SIZE)

    roll_accel = deque(maxlen=WINDOW_SIZE)
    roll_gyro = deque(maxlen=WINDOW_SIZE)
    roll_kalman = deque(maxlen=WINDOW_SIZE)

    state = {
        "pitch_gyro_integrated": 0.0,
        "roll_gyro_integrated": 0.0,
        "last_time_ms": None,
        "sample_count": 0,
    }

    fig, (ax_pitch, ax_roll) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    p_accel_line, = ax_pitch.plot([], [], label="Accel-only", alpha=0.5)
    p_gyro_line, = ax_pitch.plot([], [], label="Gyro-only", alpha=0.5)
    p_kalman_line, = ax_pitch.plot([], [], label="Kalman", linewidth=2.5, color="black")
    ax_pitch.set_title("Pitch")
    ax_pitch.set_ylabel("Degrees")
    ax_pitch.legend(loc="upper left")

    r_accel_line, = ax_roll.plot([], [], label="Accel-only", alpha=0.5)
    r_gyro_line, = ax_roll.plot([], [], label="Gyro-only", alpha=0.5)
    r_kalman_line, = ax_roll.plot([], [], label="Kalman", linewidth=2.5, color="black")
    ax_roll.set_title("Roll")
    ax_roll.set_xlabel("Sample")
    ax_roll.set_ylabel("Degrees")
    ax_roll.legend(loc="upper left")

    def read_one_sample():
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            return None
        parts = line.split(",")
        if len(parts) != 7:
            return None
        try:
            t = int(parts[0])
            ax_g, ay_g, az_g = float(parts[1]), float(parts[2]), float(parts[3])
            gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])
        except ValueError:
            return None
        return {"t": t, "ax": ax_g, "ay": ay_g, "az": az_g,
                "gx": gx, "gy": gy, "gz": gz}

    def process_sample(sample):
        if state["last_time_ms"] is None:
            dt = 0.01
        else:
            dt = (sample["t"] - state["last_time_ms"]) / 1000.0
            if dt <= 0:
                dt = 0.01
        state["last_time_ms"] = sample["t"]

        # --- PITCH: rotation using Y/Z accel split, gx gyro rate ---
        pitch_accel_angle = math.degrees(math.atan2(sample["ay"], sample["az"]))
        state["pitch_gyro_integrated"] += sample["gx"] * dt
        pitch_fused = kf_pitch.get_angle(sample["gx"], pitch_accel_angle, dt)

        # --- ROLL: rotation using X/Z accel split, gy gyro rate ---
        # NOTE: axis pairing (which accel axes + which gyro axis) depends on
        # how the sensor is physically mounted/oriented. This is the standard
        # convention, but verify empirically: tilt the sensor in the
        # direction you call "roll" and confirm this line responds to it,
        # not the pitch line. Swap gx/gy or the accel axis pair if backwards.
        roll_accel_angle = math.degrees(math.atan2(sample["ax"], sample["az"]))
        state["roll_gyro_integrated"] += sample["gy"] * dt
        roll_fused = kf_roll.get_angle(sample["gy"], roll_accel_angle, dt)

        state["sample_count"] += 1
        times.append(state["sample_count"])

        pitch_accel.append(pitch_accel_angle)
        pitch_gyro.append(state["pitch_gyro_integrated"])
        pitch_kalman.append(pitch_fused)

        roll_accel.append(roll_accel_angle)
        roll_gyro.append(state["roll_gyro_integrated"])
        roll_kalman.append(roll_fused)

    def update(frame):
        # Drain EVERYTHING currently sitting in the serial buffer, not just
        # one line. If plotting is slower than the 100Hz data rate, data
        # backs up in the buffer -- reading only one line per frame means
        # we fall further and further behind over time, which is exactly
        # the growing lag you were seeing. Processing every waiting sample
        # (even though we only draw once per frame) keeps the filters and
        # the displayed data caught up to real time.
        processed_any = False
        while ser.in_waiting > 0:
            sample = read_one_sample()
            if sample is not None:
                process_sample(sample)
                processed_any = True

        if not processed_any:
            return p_accel_line, p_gyro_line, p_kalman_line, r_accel_line, r_gyro_line, r_kalman_line

        p_accel_line.set_data(times, pitch_accel)
        p_gyro_line.set_data(times, pitch_gyro)
        p_kalman_line.set_data(times, pitch_kalman)

        r_accel_line.set_data(times, roll_accel)
        r_gyro_line.set_data(times, roll_gyro)
        r_kalman_line.set_data(times, roll_kalman)

        ax_pitch.relim(); ax_pitch.autoscale_view()
        ax_roll.relim(); ax_roll.autoscale_view()

        return p_accel_line, p_gyro_line, p_kalman_line, r_accel_line, r_gyro_line, r_kalman_line

    ani = animation.FuncAnimation(fig, update, interval=20, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()