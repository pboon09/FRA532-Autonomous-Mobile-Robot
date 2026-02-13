import numpy as np
import math


class EKF:
    X = 0
    Y = 1
    THETA = 2
    VX = 3
    VY = 4
    WZ = 5
    STATE_SIZE = 6

    def __init__(self, Q=None, P0=None):
        self.state = np.zeros(self.STATE_SIZE)

        if Q is None:
            Q = np.diag([0.001, 0.001, 0.01, 0.05, 0.05, 0.01])
        self.Q = np.array(Q) if isinstance(Q, list) else Q
        if self.Q.ndim == 1:
            self.Q = np.diag(self.Q)

        if P0 is None:
            P0 = np.diag([0.01, 0.01, 0.1, 0.5, 0.5, 0.1])
        self.P = np.array(P0) if isinstance(P0, list) else P0
        if self.P.ndim == 1:
            self.P = np.diag(self.P)

        self.history = {
            'state': [],
            'P': [],
            'K': [],
            'innovation': []
        }

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def predict(self, v, omega, dt):
        theta = self.state[self.THETA]

        theta_new = self.normalize_angle(theta + omega * dt)
        cos_theta_new = math.cos(theta_new)
        sin_theta_new = math.sin(theta_new)

        vx_new = v * cos_theta_new
        vy_new = v * sin_theta_new

        delta_theta = theta_new - theta
        if abs(delta_theta) > math.pi:
            delta_theta = delta_theta - 2 * math.pi * np.sign(delta_theta)
        wz_new = delta_theta / dt if dt > 1e-6 else omega

        self.state[self.X] += vx_new * dt
        self.state[self.Y] += vy_new * dt
        self.state[self.THETA] = theta_new
        self.state[self.VX] = vx_new
        self.state[self.VY] = vy_new
        self.state[self.WZ] = wz_new

        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        F = np.eye(self.STATE_SIZE)
        F[self.X, self.THETA] = -v * sin_theta * dt
        F[self.Y, self.THETA] = v * cos_theta * dt
        F[self.VX, self.THETA] = -v * sin_theta_new
        F[self.VY, self.THETA] = v * cos_theta_new

        self.P = F @ self.P @ F.T + self.Q

    def correct(self, z, R):
        H = np.array([[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])

        y = z - H @ self.state
        y[0] = self.normalize_angle(y[0])

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.state = self.state + (K @ y).flatten()
        self.state[self.THETA] = self.normalize_angle(self.state[self.THETA])

        I = np.eye(self.STATE_SIZE)
        self.P = (I - K @ H) @ self.P

        self.history['state'].append(self.state.copy())
        self.history['P'].append(self.P.copy())
        self.history['K'].append(K.copy())
        self.history['innovation'].append(y[0])

        return K[self.THETA, 0], y[0], S[0, 0]

    def get_state(self):
        return self.state.copy()

    def get_covariance(self):
        return self.P.copy()

    def reset(self, Q=None, P0=None):
        self.state = np.zeros(self.STATE_SIZE)

        if Q is not None:
            self.Q = np.array(Q) if isinstance(Q, list) else Q
            if self.Q.ndim == 1:
                self.Q = np.diag(self.Q)

        if P0 is not None:
            self.P = np.array(P0) if isinstance(P0, list) else P0
            if self.P.ndim == 1:
                self.P = np.diag(self.P)
        else:
            self.P = np.diag([0.01, 0.01, 0.1, 0.5, 0.5, 0.1])

        self.history = {
            'state': [],
            'P': [],
            'K': [],
            'innovation': []
        }
