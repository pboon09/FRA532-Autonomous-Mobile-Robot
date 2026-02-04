import math


class WheelOdometry:
    def __init__(self, wheel_radius=0.033, track_width=0.160):
        self.r = wheel_radius
        self.b = track_width

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_rad_l = None
        self.last_rad_r = None

        self.v = 0.0
        self.omega = 0.0

        self.history = {
            'x': [],
            'y': [],
            'theta': [],
            'v': [],
            'omega': []
        }

    def update(self, rad_l, rad_r, dt):
        if self.last_rad_l is None:
            self.last_rad_l = rad_l
            self.last_rad_r = rad_r
            return False

        delta_rad_l = rad_l - self.last_rad_l
        delta_rad_r = rad_r - self.last_rad_r

        d_l = delta_rad_l * self.r
        d_r = delta_rad_r * self.r

        d_theta = (d_r - d_l) / self.b

        if abs(d_r - d_l) < 1e-6:
            d_center = (d_l + d_r) / 2.0
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)
        else:
            R = (self.b / 2.0) * (d_l + d_r) / (d_r - d_l)

            icc_x = self.x - R * math.sin(self.theta)
            icc_y = self.y + R * math.cos(self.theta)

            cos_dtheta = math.cos(d_theta)
            sin_dtheta = math.sin(d_theta)

            new_x = cos_dtheta * (self.x - icc_x) - sin_dtheta * (self.y - icc_y) + icc_x
            new_y = sin_dtheta * (self.x - icc_x) + cos_dtheta * (self.y - icc_y) + icc_y

            self.x = new_x
            self.y = new_y

        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        if dt > 0:
            self.v = (d_l + d_r) / (2.0 * dt)
            self.omega = (d_r - d_l) / (self.b * dt)

        self.last_rad_l = rad_l
        self.last_rad_r = rad_r

        self.history['x'].append(self.x)
        self.history['y'].append(self.y)
        self.history['theta'].append(self.theta)
        self.history['v'].append(self.v)
        self.history['omega'].append(self.omega)

        return True

    def get_pose(self):
        return self.x, self.y, self.theta

    def get_velocity(self):
        return self.v, self.omega

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_rad_l = None
        self.last_rad_r = None
        self.v = 0.0
        self.omega = 0.0
        self.history = {
            'x': [],
            'y': [],
            'theta': [],
            'v': [],
            'omega': []
        }
