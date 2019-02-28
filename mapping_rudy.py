import matplotlib.pyplot as plt
import urg_lidar
import logs
import math
from logs import File
from scipy.interpolate import interp1d as interpolate 


def extract_col(matrix, column_no): return [row[column_no] for row in matrix]


if __name__ == "__main__":
    lidar = urg_lidar.UrgLidar()

    logs = logs.Logs(lidar, File.ONE_LOOP_ANTI_CLOCKWISE)
    urg_scans = logs.get_lidar_scans(radians=True)
    robot_trajectory = logs.get_robot_trajectory()

    x_robot = interpolate(extract_col(robot_trajectory, 0), extract_col(robot_trajectory, 1))
    y_robot = interpolate(extract_col(robot_trajectory, 0), extract_col(robot_trajectory, 2))
    theta_robot = interpolate(extract_col(robot_trajectory, 0), extract_col(robot_trajectory, 3))
    
    point_cloud = []

    i = 0
    min_time, max_time = min(extract_col(robot_trajectory, 0)), max(extract_col(robot_trajectory, 0))
    for timestamp, rhos in urg_scans:
        if not min_time <= timestamp <= max_time: continue
        i += 1
        theta = theta_robot(timestamp)
        x, y = x_robot(timestamp), y_robot(timestamp)
        d_x_lidar, d_y_lidar = lidar.offset / 1000 * math.cos(theta), lidar.offset / 1000 * math.sin(theta)
        # d_x_lidar, d_y_lidar = 0, 0
        x_lidar, y_lidar = x + d_x_lidar, y + d_y_lidar

        for distance, angle in rhos:
            if distance == 0: continue
            rho = angle + theta
            point_cloud.append((x_lidar + distance / 1000 * math.cos(rho), y_lidar + distance / 1000 * math.sin(rho)))
        if i == 10:
            i = 0
            # plt.clf()
            # plt.scatter(extract_col(point_cloud, 0), extract_col(point_cloud, 1), s=0.01, c="r")
            # plt.plot(extract_col(robot_trajectory, 1), extract_col(robot_trajectory, 2), c="g")
            # plt.scatter(x, y, s=20, c="b", marker="o")
            # plt.pause(0.01)

    plt.clf()
    plt.scatter(extract_col(point_cloud, 0), extract_col(point_cloud, 1), s=0.01, c="r")
    plt.plot(extract_col(robot_trajectory, 1), extract_col(robot_trajectory, 2), c="g")
    plt.show()
