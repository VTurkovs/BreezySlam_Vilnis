import breezyslam.algorithms as SLAM
import math, logs, urg_lidar
from logs import File
from datetime import datetime
from time import sleep
from scipy.interpolate import interp1d as interpolate
from PIL import Image
from roboviz import MapVisualizer
from threading import Thread


MAP_SIZE_PIXELS = 1000
MAP_SIZE_METERS = 20


def extract_col(matrix, column_no): return [row[column_no] for row in matrix]


def get_robot_pose_change(prev_time, cur_time, x_robot, y_robot, theta_robot):
    prev_x, prev_y, prev_theta = x_robot(prev_time), y_robot(prev_time), theta_robot(prev_time)
    cur_x, cur_y, cur_theta = x_robot(cur_time), y_robot(cur_time), theta_robot(cur_time)
    dxy_mm = math.sqrt((cur_x - prev_x) ** 2 + (cur_y - prev_y) ** 2) * 1000
    d_theta_deg = math.degrees(cur_theta - prev_theta)
    dt_seconds = cur_time - prev_time
    return dxy_mm, d_theta_deg, dt_seconds


def thread_func(robot_path, slam, lidar_scans, mapbytes, pose):
    min_time, max_time = min(extract_col(robot_path, 0)), max(extract_col(robot_path, 0))
    x_robot_interp = interpolate(extract_col(robot_path, 0), extract_col(robot_path, 1), kind="cubic")
    y_robot_interp = interpolate(extract_col(robot_path, 0), extract_col(robot_path, 2), kind="cubic")
    theta_robot_interp = interpolate(extract_col(robot_path, 0), extract_col(robot_path, 3), kind="cubic")

    last_time = 0
    for timestamp, scans in lidar_scans:
        if not min_time <= timestamp <= max_time: continue
        if last_time == 0:
            last_time = timestamp
            continue

        scans_mm = extract_col(scans, 0)

        pose_change = get_robot_pose_change(last_time, timestamp, x_robot_interp, y_robot_interp, theta_robot_interp)
        slam.update(scans_mm, pose_change)
        pose[0], pose[1], pose[2] = slam.getpos()
        slam.getmap(mapbytes)

        if last_time > 0: sleep((timestamp - last_time) / 3)
        last_time = timestamp


if __name__ == "__main__":
    lidar = urg_lidar.UrgLidar()

    logs = logs.Logs(lidar, File.ONE_LOOP_ANTI_CLOCKWISE)
    urg_scans = logs.get_lidar_scans()
    robot_trajectory = logs.get_robot_trajectory()

    seed = int((datetime.now() - datetime(1970, 1, 1)).total_seconds())

    # slam = SLAM.Deterministic_SLAM(
    #     lidar.breezy_lidar(),
    #     MAP_SIZE_PIXELS,
    #     MAP_SIZE_METERS,
    #     map_quality=255,
    #     hole_width_mm=100)

    slam = SLAM.RMHC_SLAM(
        lidar.breezy_lidar(),
        MAP_SIZE_PIXELS,
        MAP_SIZE_METERS,
        map_quality=50,
        hole_width_mm=100,
        random_seed=seed,
        sigma_xy_mm=30,
        sigma_theta_degrees=5,
        max_search_iter=1000)

    # slam = SLAM.RMHC_SLAM(
    #     lidar.breezy_lidar(),
    #     MAP_SIZE_PIXELS,
    #     MAP_SIZE_METERS,
    #     map_quality=50,
    #     hole_width_mm=200,
    #     random_seed=seed,
    #     sigma_xy_mm=30,
    #     sigma_theta_degrees=5,
    #     max_search_iter=500)

    map_bytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    pose = [0, 0, 0]
    thread = Thread(target=thread_func, args=(robot_trajectory, slam, urg_scans, map_bytes, pose))
    thread.daemon = True
    thread.start()

    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, title=logs.file, show_trajectory=True)
    while True:
        if not viz.display(pose[0]/1000, pose[1]/1000, pose[2], map_bytes): break

    # Save map and trajectory as PNG file
    # image = Image.frombuffer('L', (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), map_bytes, 'raw', 'L', 0, 1)
    # image.show()
    # image.save('%s.png' % logs.file)

