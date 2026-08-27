"""
kalman.py

A minimal 2-state Kalman filter for fusing gyroscope + accelerometer
readings into a single, drift-free, low-noise angle estimate.

State vector: [angle, gyro_bias]
  - angle:     our best estimate of the true tilt angle (degrees)
  - gyro_bias: our best estimate of the gyro's constant offset error (deg/s)

This is the classic minimal formulation used in balancing-robot and
IMU tutorials (e.g. Kristian Lauszus' well-known Arduino Kalman filter).
Kept deliberately small and single-purpose so every line can be explained
and defended -- no black-box library, just the raw predict/update math.
"""


class KalmanFilter1D:
    def __init__(self, q_angle=0.001, q_bias=0.003, r_measure=0.03):
        """
        Tuning parameters:
          q_angle    - process noise for the angle itself. Higher = trust
                       the motion model less, respond to accel faster.
          q_bias     - process noise for how fast the gyro's bias can drift.
                       Higher = allow the bias estimate to change faster.
          r_measure  - measurement noise for the accelerometer-derived angle.
                       Higher = trust the accelerometer less (smoother,
                       slower to correct); lower = trust it more (faster
                       correction, noisier output).
        """
        self.q_angle = q_angle
        self.q_bias = q_bias
        self.r_measure = r_measure

        self.angle = 0.0   # current angle estimate (degrees)
        self.bias = 0.0    # current gyro bias estimate (deg/s)

        # 2x2 error covariance matrix -- represents our uncertainty in
        # [angle, bias] and how those uncertainties are correlated.
        # Starts at zero; it will grow/settle naturally as the filter runs.
        self.P = [[0.0, 0.0], [0.0, 0.0]]

    def get_angle(self, new_rate, new_angle, dt):
        """
        Run one predict+update cycle of the filter.

        new_rate  - current gyro reading (deg/s)
        new_angle - current accelerometer-derived angle (degrees), from atan2
        dt        - time elapsed since the last call, in seconds

        Returns the fused angle estimate (degrees).
        """

        # ---- PREDICT step ----
        # Use the gyro (minus our current bias estimate) to project the
        # angle forward by one time step. Same integration idea as the
        # gyro-only drift demo -- but here it's just a short-term guess
        # that the update step will immediately correct.
        rate = new_rate - self.bias
        self.angle += dt * rate

        # Propagate the uncertainty (covariance) forward too -- our
        # confidence in the estimate naturally decreases a little each
        # predict step, and Q_angle/Q_bias control how much.
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.q_bias * dt

        # ---- UPDATE step ----
        # Compare our predicted angle to what the accelerometer says right
        # now. The difference ("innovation") is the signal we use to
        # correct both the angle and the bias estimate.
        S = self.P[0][0] + self.r_measure  # innovation covariance
        K0 = self.P[0][0] / S              # Kalman gain for angle
        K1 = self.P[1][0] / S              # Kalman gain for bias

        y = new_angle - self.angle         # innovation (measurement residual)

        self.angle += K0 * y
        self.bias += K1 * y

        # Update covariance to reflect the new, tighter certainty after
        # incorporating this measurement.
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K0 * P00_temp
        self.P[0][1] -= K0 * P01_temp
        self.P[1][0] -= K1 * P00_temp
        self.P[1][1] -= K1 * P01_temp

        return self.angle


if __name__ == "__main__":
    # Tiny sanity check you can run directly: python3 kalman.py
    # Simulates a sensor sitting still with a small constant gyro bias
    # and noisy accelerometer readings, and shows the filter converging.
    import random

    kf = KalmanFilter1D()
    true_angle = 0.0
    simulated_bias = 1.5  # deg/s of fake constant gyro error
    dt = 0.01

    print("step  gyro_only(drift)  accel_only(noisy)  kalman(fused)")
    gyro_only = 0.0
    for step in range(300):
        noisy_rate = simulated_bias + random.uniform(-0.5, 0.5)
        noisy_accel_angle = true_angle + random.uniform(-3, 3)

        gyro_only += noisy_rate * dt
        fused = kf.get_angle(noisy_rate, noisy_accel_angle, dt)

        if step % 50 == 0:
            print(f"{step:4d}  {gyro_only:16.2f}  {noisy_accel_angle:17.2f}  {fused:13.2f}")
