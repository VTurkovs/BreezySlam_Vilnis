import math
import csv
from enum import Enum


class File(Enum):
    ONE_LOOP_CLOCKWISE = 0
    ONE_LOOP_ANTI_CLOCKWISE = 1
    MANY_LOOPS = 2
    ROTATION = 3
    THREE_ROTATIONS = 4
    COVERAGE = 5
    RANDOM = 6


N: float = 1.057613e-05
G: float = 3.659689e-07


class Logs:
    def __init__(self, lidar, file: File):
        if file == File.ONE_LOOP_CLOCKWISE: self.file = "one_loop_clockwise.log"
        elif file == File.ONE_LOOP_ANTI_CLOCKWISE: self.file = "one_loop_anti_clockwise.log"
        elif file == File.MANY_LOOPS: self.file = "many_loops.log"
        elif file == File.ROTATION: self.file = "rotation.log"
        elif file == File.THREE_ROTATIONS: self.file = "three_rotations.log"
        elif file == File.COVERAGE: self.file = "coverage.log"
        elif file == File.RANDOM: self.file = "random.log"

        self.directory = "./logs"
        self.urg_lidar = lidar

    def get_robot_trajectory(self):
        log_file_path = f"{self.directory}/reactd_{self.file}"
        with open(log_file_path) as log_file:
            reader = csv.DictReader(log_file)

            robot_pos = []
            timestamp, x_robot, y_robot = 0, 0, 0

            gyro_error, gyro_error_sum, gyro_error_count = 0.0, 0.0, 0

            d_s, d_theta_robot = 0.0, 0.0
            x_robot, y_robot, theta_robot = 0.0, 0.0, 0.0

            for row in reader:
                timestamp += float(row['Timestamp'])
                gyro = float(row['gyro'])
                left_encoder, right_encoder = float(row['leftEncoder']), float(row['rightEncoder'])
                if left_encoder == 0.0 and right_encoder == 0.0:
                    gyro_error_sum += gyro
                    gyro_error_count += 1
                else:
                    gyro_error = gyro_error_sum / gyro_error_count

                theta_robot += d_theta_robot
                x_robot += d_s * math.cos(theta_robot + d_theta_robot / 2)
                y_robot += d_s * math.sin(theta_robot + d_theta_robot / 2)
                robot_pos.append((timestamp, x_robot, y_robot, theta_robot))

                d_theta_robot = (gyro - gyro_error) * G
                d_s = (left_encoder + right_encoder) * N / 2
        return robot_pos

    def get_lidar_scans(self, radians=False):
        log_file_path = f"{self.directory}/lidar_{self.file}"
        scans = []
        with open(log_file_path) as log_file:
            reader = csv.reader(log_file)
            timestamp = 0
            zeroth_step = self.urg_lidar.zeroth_step_rad if radians else self.urg_lidar.zeroth_step_deg
            step_resolution = self.urg_lidar.step_resolution_rad if radians else self.urg_lidar.step_resolution_deg
            for row in reader:
                if "Timestamp" in row: row = next(reader)
                timestamp += float(row[0])
                first_step = int(row[1])
                angle = zeroth_step + step_resolution * first_step
                rhos = []
                for distance in row[3:]:
                    dist = int(distance)
                    rhos.append((dist if 4000 > dist > 60 else 0, angle))
                    # rhos.append((dist, angle))
                    angle += step_resolution

                # for i in range(3, len(row)):
                #     distance = int(row[i])
                #     # if 4095 < distance or distance < 60: distance = 0
                #     if distance == 4095: distance = 0
                #     rhos.append((distance, angle))
                #     angle += self.urg_lidar.step_resolution_deg
                scans.append((timestamp, rhos))
        return scans

