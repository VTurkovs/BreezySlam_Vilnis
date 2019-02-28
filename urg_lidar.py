import math
import breezyslam.sensors as lidar


class UrgLidar:
    def __init__(self):
        self.first_step, self.last_step = 128, 640
        self.scan_size = self.last_step - self.first_step + 1
        self.step_resolution_rad: float = 2 * math.pi / 1024
        self.step_resolution_deg: float = 360 / 1024
        self.zeroth_step_deg: float = -135
        self.zeroth_step_rad: float = self.zeroth_step_deg * math.pi / 180
        self.update_rate = 10  # Hz
        self.offset = 350  # mm

    def all_angles(self, radians=False):
        if radians:
            return [self.zeroth_step_rad + step * self.step_resolution_rad for step in
                    range(self.first_step, self.last_step)]
        else:
            return [self.zeroth_step_deg + step * self.step_resolution_deg for step in
                    range(self.first_step, self.last_step)]

    def scan_angle(self, radians=False):
        all_angles = self.all_angles(radians)
        return max(all_angles) - min(all_angles)

    def breezy_lidar(self):
        return lidar.Laser(self.scan_size, self.update_rate, self.scan_angle(), 4095, 60, offset_mm=self.offset)
