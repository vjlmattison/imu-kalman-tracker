
"""
serial_reader_2axis.py

Same three-way comparison as live_plot_2axis.py (accel-only, gyro-only,
Kalman-fused, for both pitch and roll) but printed as scrolling text
instead of a live graph -- easier to watch react instantly, no plotting
window to manage.
"""

import math
import serial

from kalman import KalmanFilter1D

SERIAL_PORT = "COM5"   # EDIT to match your Arduino's port
BAUD_RATE = 115200

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Listening on {SERIAL_PORT}...")

    kf_pitch = KalmanFilter1D(q_angle=0.001, q_bias=0.003, r_measure=0.03)
    kf_roll = KalmanFilter1D(q_angle=0.001, q_bias=0.003, r_measure=0.03)

    pitch_gyro_integrated = 0.0
    roll_gyro_integrated = 0.0
    last_time_ms = None
    last_sent_angle = None  # tracks the last angle actually sent to the servo
    SERVO_THRESHOLD = 1.0   # degrees -- only send a new command if the change exceeds this
    last_print_time = 0.0   # tracks last time we printed, for the 1-per-second throttle

    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        parts = line.split(",")
        if len(parts) != 7:
            continue
        try:
            t = int(parts[0])
            ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
            gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])
        except ValueError:
            continue

        if last_time_ms is None:
            dt = 0.01
        else:
            dt = (t - last_time_ms) / 1000.0
            if dt <= 0:
                dt = 0.01
        last_time_ms = t

        pitch_accel = math.degrees(math.atan2(ay, az))
        pitch_gyro_integrated += gx * dt
        pitch_fused = kf_pitch.get_angle(gx, pitch_accel, dt)

        roll_accel = math.degrees(math.atan2(ax, az))
        roll_gyro_integrated += gy * dt
        roll_fused = kf_roll.get_angle(gy, roll_accel, dt)

        # Print roughly once per second instead of every sample (100/sec) --
        # the filter still runs on every sample underneath, this only
        # throttles what gets printed to the screen.
        if t - last_print_time >= 1000:
            last_print_time = t
            print(f"PITCH  accel={pitch_accel:7.2f}  gyro={pitch_gyro_integrated:7.2f}  kalman={pitch_fused:7.2f}   |   "
                  f"ROLL  accel={roll_accel:7.2f}  gyro={roll_gyro_integrated:7.2f}  kalman={roll_fused:7.2f}")

        # --- Send the Kalman-fused PITCH angle back to the Arduino to drive the servo ---
        # Only send a new command if the angle has moved enough to matter --
        # otherwise tiny noise-level fluctuations (even after filtering)
        # cause the servo to constantly micro-correct and buzz/chatter
        # instead of sitting still. This is a simple deadband/threshold.
        if last_sent_angle is None or abs(pitch_fused - last_sent_angle) >= SERVO_THRESHOLD:
            command = f"{pitch_fused:.2f}\n"
            ser.write(command.encode("utf-8"))
            last_sent_angle = pitch_fused

if __name__ == "__main__":
    main()



# ____________________OLDER VERSIONS___________________________________________

# """
# serial_reader_2axis.py

# Same three-way comparison as live_plot_2axis.py (accel-only, gyro-only,
# Kalman-fused, for both pitch and roll) but printed as scrolling text
# instead of a live graph -- easier to watch react instantly, no plotting
# window to manage.
# """

# import math
# import serial

# from kalman import KalmanFilter1D

# SERIAL_PORT = "COM5"   # EDIT to match your Arduino's port
# BAUD_RATE = 115200

# def main():
#     ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
#     print(f"Listening on {SERIAL_PORT}...")

#     kf_pitch = KalmanFilter1D(q_angle=0.001, q_bias=0.003, r_measure=0.03)
#     kf_roll = KalmanFilter1D(q_angle=0.001, q_bias=0.003, r_measure=0.03)

#     pitch_gyro_integrated = 0.0
#     roll_gyro_integrated = 0.0
#     last_time_ms = None
#     last_sent_angle = None  # tracks the last angle actually sent to the servo
#     SERVO_THRESHOLD = 1.0   # degrees -- only send a new command if the change exceeds this

#     while True:
#         line = ser.readline().decode("utf-8", errors="ignore").strip()
#         if not line:
#             continue
#         parts = line.split(",")
#         if len(parts) != 7:
#             continue
#         try:
#             t = int(parts[0])
#             ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
#             gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])
#         except ValueError:
#             continue

#         if last_time_ms is None:
#             dt = 0.01
#         else:
#             dt = (t - last_time_ms) / 1000.0
#             if dt <= 0:
#                 dt = 0.01
#         last_time_ms = t

#         pitch_accel = math.degrees(math.atan2(ay, az))
#         pitch_gyro_integrated += gx * dt
#         pitch_fused = kf_pitch.get_angle(gx, pitch_accel, dt)

#         roll_accel = math.degrees(math.atan2(ax, az))
#         roll_gyro_integrated += gy * dt
#         roll_fused = kf_roll.get_angle(gy, roll_accel, dt)

#         print(f"PITCH  accel={pitch_accel:7.2f}  gyro={pitch_gyro_integrated:7.2f}  kalman={pitch_fused:7.2f}   |   "
#               f"ROLL  accel={roll_accel:7.2f}  gyro={roll_gyro_integrated:7.2f}  kalman={roll_fused:7.2f}")

#         # --- Send the Kalman-fused PITCH angle back to the Arduino to drive the servo ---
#         # Only send a new command if the angle has moved enough to matter --
#         # otherwise tiny noise-level fluctuations (even after filtering)
#         # cause the servo to constantly micro-correct and buzz/chatter
#         # instead of sitting still. This is a simple deadband/threshold.
#         if last_sent_angle is None or abs(pitch_fused - last_sent_angle) >= SERVO_THRESHOLD:
#             command = f"{pitch_fused:.2f}\n"
#             ser.write(command.encode("utf-8"))
#             last_sent_angle = pitch_fused

# if __name__ == "__main__":
#     main()


# """
# serial_reader_2axis_logging.py

# Same as serial_reader_2axis.py, but also writes every sample to a CSV file
# so a long test run (5+ minutes) can be plotted and reviewed afterward
# instead of relying on watching scrolling text the whole time.

# This is specifically for the soldered-board re-test: confirming whether
# the earlier roll-axis drift finding (seen at high R on the old, unsoldered
# board) was a real gyro bias issue or an artifact of a flaky connection.
# """

# import math
# import csv
# import time
# import serial

# from kalman import KalmanFilter1D

# SERIAL_PORT = "COM5"   # EDIT to match your Arduino's port
# BAUD_RATE = 115200

# # EDIT this if you want a specific R value for this test run (e.g. 0.5 to
# # match the earlier high-R condition that showed roll drift)
# R_MEASURE = 0.03

# LOG_FILENAME = "kalman_long_test_log.csv"

# def main():
#     ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
#     print(f"Listening on {SERIAL_PORT}...")
#     print(f"Logging to {LOG_FILENAME} (R = {R_MEASURE}). Press Ctrl+C to stop.")

#     kf_pitch = KalmanFilter1D(q_angle=0.1, q_bias=0.003, r_measure=R_MEASURE)
#     kf_roll = KalmanFilter1D(q_angle=0.1, q_bias=0.003, r_measure=R_MEASURE)

#     pitch_gyro_integrated = 0.0
#     roll_gyro_integrated = 0.0
#     last_time_ms = None

#     start_time = time.time()

#     with open(LOG_FILENAME, "w", newline="") as csvfile:
#         writer = csv.writer(csvfile)
#         writer.writerow([
#             "elapsed_sec", "pitch_accel", "pitch_gyro", "pitch_kalman",
#             "roll_accel", "roll_gyro", "roll_kalman"
#         ])

#         while True:
#             line = ser.readline().decode("utf-8", errors="ignore").strip()
#             if not line:
#                 continue
#             parts = line.split(",")
#             if len(parts) != 7:
#                 continue
#             try:
#                 t = int(parts[0])
#                 ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
#                 gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])
#             except ValueError:
#                 continue

#             if last_time_ms is None:
#                 dt = 0.01
#             else:
#                 dt = (t - last_time_ms) / 1000.0
#                 if dt <= 0:
#                     dt = 0.01
#             last_time_ms = t

#             pitch_accel = math.degrees(math.atan2(ay, az))
#             pitch_gyro_integrated += gx * dt
#             pitch_fused = kf_pitch.get_angle(gx, pitch_accel, dt)

#             roll_accel = math.degrees(math.atan2(ax, az))
#             roll_gyro_integrated += gy * dt
#             roll_fused = kf_roll.get_angle(gy, roll_accel, dt)

#             elapsed = time.time() - start_time

#             writer.writerow([
#                 f"{elapsed:.2f}",
#                 f"{pitch_accel:.3f}", f"{pitch_gyro_integrated:.3f}", f"{pitch_fused:.3f}",
#                 f"{roll_accel:.3f}", f"{roll_gyro_integrated:.3f}", f"{roll_fused:.3f}",
#             ])
#             csvfile.flush()  # force this row to disk immediately -- don't rely on
#                               # Python's internal buffer, so Ctrl+C never loses data

#             # Print a status line every ~1 second instead of every single
#             # sample (100/sec would flood the terminal) -- just enough to
#             # confirm it's alive and see rough values while it runs.
#             if int(elapsed) != int(elapsed - dt):
#                 print(f"[{elapsed:6.1f}s]  PITCH kalman={pitch_fused:7.2f}   ROLL kalman={roll_fused:7.2f}")

# if __name__ == "__main__":
#     main()