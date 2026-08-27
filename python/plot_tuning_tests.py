"""
plot_tuning_tests.py

Generates 3-line comparison plots (accel-only, gyro-only, Kalman) for
pitch and roll, from the tilt-and-hold tuning test CSVs. One PNG per
setting -- these are the README's tuning-table visual evidence.
"""

import pandas as pd
import matplotlib.pyplot as plt

FILES = {
    "r=0.005tilted.csv": "R = 0.005 (low)",
    "r=0.03tilted.csv": "R = 0.03 (default)",
    "r=0.5tilted.csv": "R = 0.5 (high)",
    "q=0.1tilted.csv": "Q_angle = 0.1 (high)",
}

for filename, label in FILES.items():
    df = pd.read_csv(filename)

    fig, (ax_pitch, ax_roll) = plt.subplots(2, 1, figsize=(11, 8), sharex=True)

    ax_pitch.plot(df["elapsed_sec"], df["pitch_accel"], alpha=0.5, label="Accel-only", color="tab:blue")
    ax_pitch.plot(df["elapsed_sec"], df["pitch_gyro"], alpha=0.6, label="Gyro-only", color="tab:orange")
    ax_pitch.plot(df["elapsed_sec"], df["pitch_kalman"], linewidth=2, label="Kalman", color="black")
    ax_pitch.set_title(f"Pitch — {label}")
    ax_pitch.set_ylabel("Degrees")
    ax_pitch.legend(loc="upper left")
    ax_pitch.grid(alpha=0.3)

    ax_roll.plot(df["elapsed_sec"], df["roll_accel"], alpha=0.5, label="Accel-only", color="tab:blue")
    ax_roll.plot(df["elapsed_sec"], df["roll_gyro"], alpha=0.6, label="Gyro-only", color="tab:orange")
    ax_roll.plot(df["elapsed_sec"], df["roll_kalman"], linewidth=2, label="Kalman", color="black")
    ax_roll.set_title(f"Roll — {label}")
    ax_roll.set_xlabel("Elapsed time (seconds)")
    ax_roll.set_ylabel("Degrees")
    ax_roll.legend(loc="upper left")
    ax_roll.grid(alpha=0.3)

    fig.suptitle(f"Tilt-and-hold tuning test — {label}\n(soldered board — flat/left/flat/right/flat/forward/flat/back/flat)")
    plt.tight_layout()

    out_name = filename.replace(".csv", "_plot.png")
    plt.savefig(out_name, dpi=150)
    plt.close(fig)
    print(f"Saved {out_name}")

print("Done.")
