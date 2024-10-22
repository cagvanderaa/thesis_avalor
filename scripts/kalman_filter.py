# kalman_filter.py

import numpy as np
from filterpy.kalman import KalmanFilter
import copy

class KalmanFilterPredictor:
    def __init__(self, dt=1/30, 
                 process_noise_std=0.1, 
                 measurement_noise_std=5.0,
                 initial_state=None,
                 initial_covariance=None):
        """
        Initializes the Kalman Filter for predicting target position in image space.

        Parameters:
        - dt (float): Time step between predictions (seconds).
        - process_noise_std (float): Standard deviation of the process noise.
        - measurement_noise_std (float): Standard deviation of the measurement noise.
        - initial_state (np.array): Initial state vector [u, v, du, dv].
        - initial_covariance (np.array): Initial covariance matrix.
        """
        self.dt = dt
        self.kf = KalmanFilter(dim_x=4, dim_z=2)

        # State Transition Matrix (F)
        self.kf.F = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Observation Matrix (H)
        self.kf.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # Initial Covariance Matrix (P)
        if initial_covariance is not None:
            self.kf.P = initial_covariance
        else:
            self.kf.P = np.array([
                [1000, 0, 0, 0],
                [0, 1000, 0, 0],
                [0, 0, 100, 0],
                [0, 0, 0, 100]
            ])

        # Process Noise Covariance (Q)
        q = process_noise_std

        self.kf.Q = np.array([
            [0.00003, 0, 0, 0],
            [0, 0.00003, 0, 0],
            [0, 0, 0.111, 0],
            [0, 0, 0, 0.111]
        ])

        # Measurement Noise Covariance (R)
        r = measurement_noise_std
        self.kf.R = np.array([
            [r**2, 0],
            [0, r**2]
        ])

        # Initial State (x)
        if initial_state is not None:
            self.kf.x = initial_state
        else:
            self.kf.x = np.array([0, 0, 0, 0])  # [u, v, du, dv]

        self.is_initialized = False

    def initialize(self, measurement):
        """
        Initializes the filter with the first measurement.

        Parameters:
        - measurement (np.array): First measurement [u, v].
        """
        self.kf.x[:2] = measurement
        self.is_initialized = True

    def predict(self):
        """
        Performs the prediction step of the Kalman Filter.
        """
        self.kf.predict()

    def update(self, measurement):
        """
        Performs the update step of the Kalman Filter with the latest measurement.

        Parameters:
        - measurement (np.array): Latest measurement [u, v].
        """
        self.kf.update(measurement)

    def get_predicted_position(self, steps_ahead=1):
        """
        Predicts the future position by a specified number of steps ahead.

        Parameters:
        - steps_ahead (int): Number of steps to predict ahead.

        Returns:
        - predicted_pos (np.array): Predicted position [u, v].
        """
        u, v, du, dv = self.kf.x

        # Compute predicted position
        predicted_u = u + du * self.dt * steps_ahead
        predicted_v = v + dv * self.dt * steps_ahead

        return np.array([predicted_u, predicted_v])
