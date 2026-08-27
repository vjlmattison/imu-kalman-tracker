"""
plot_drift_tests.py

Generates and saves comparison plots from the long stillness-test CSVs
(logged by serial_reader_2axis_logging.py). One PNG per test file, each
showing pitch and roll Kalman estimates over the full run -- this is the
evidence behind the "roll drift did not reproduce on the soldered board"
finding in the README.

Usage: put this script in the same folder as your CSV files, then run:
    python3 plot_drift_tests.py
Edit the FILES dict below to match your actual filenames/labels.
"""

import pandas as pd
import matplotlib.pyplot as plt

# Map each CSV filename to a readable label for the plot title/legend.
# EDIT these to match your actual saved filenames.
FILES = {
    "r=0.03.csv": "R = 0.03 (default)",
    "r=0.5.csv": "R = 0.5 (high)",
    "q=0.1.csv": "Q_angle = 0.1 (high)",
}

for filename, label in FILES.items():
    df = pd.read_csv(filename)

    fig, (ax_pitch, ax_roll) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax_pitch.plot(df["elapsed_sec"], df["pitch_kalman"], color="black", linewidth=1)
    ax_pitch.set_title(f"Pitch — {label}")
    ax_pitch.set_ylabel("Degrees")
    ax_pitch.grid(alpha=0.3)

    ax_roll.plot(df["elapsed_sec"], df["roll_kalman"], color="black", linewidth=1)
    ax_roll.set_title(f"Roll — {label}")
    ax_roll.set_xlabel("Elapsed time (seconds)")
    ax_roll.set_ylabel("Degrees")
    ax_roll.grid(alpha=0.3)

    fig.suptitle(f"Stationary stability test — {label}\n(soldered board, {df['elapsed_sec'].iloc[-1]:.0f}s run)")
    plt.tight_layout()

    out_name = filename.replace(".csv", "_stability_plot.png")
    plt.savefig(out_name, dpi=150)
    plt.close(fig)
    print(f"Saved {out_name}")

print("Done.")
