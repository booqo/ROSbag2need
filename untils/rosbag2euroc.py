# -*- coding: utf-8 -*-
import rosbag
import csv
from sensor_msgs.msg import Imu, Image
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import shutil
import numpy as np
import yaml


def create_euroc_dirs(base_folder='Dataset'):
    """
    创建EuRoC所需的目录结构。

    :param base_folder: 基础文件夹名称，默认是'Dataset'
    """
    cam0_data = os.path.join(base_folder, 'cam0', 'data')
    cam1_data = os.path.join(base_folder, 'cam1', 'data')
    imu0_folder = os.path.join(base_folder, 'imu0')

    # 创建cam0/data目录
    os.makedirs(cam0_data, exist_ok=True)
    print(f"创建目录: {cam0_data}")

    # 创建cam1/data目录
    os.makedirs(cam1_data, exist_ok=True)
    print(f"创建目录: {cam1_data}")

    # 创建imu0目录
    os.makedirs(imu0_folder, exist_ok=True)
    print(f"创建目录: {imu0_folder}")


def load_camera_parameters(calibration_file):
    """
    从YAML格式的校准文件中加载相机内参和畸变系数。

    :param calibration_file: 校准文件的路径
    :return: 相机矩阵和畸变系数
    """
    with open(calibration_file, 'r') as file:
        calib_data = yaml.safe_load(file)

    # 假设校准文件中包含 'camera_matrix' 和 'distortion_coefficients'
    camera_matrix = calib_data['camera_matrix']['data']
    distortion_coefficients = calib_data['distortion_coefficients']['data']

    # 转换为NumPy数组
    camera_matrix = np.array(camera_matrix).reshape(3, 3).astype(float)
    distortion_coefficients = np.array(distortion_coefficients).reshape(1, -1).astype(float)

    return camera_matrix, distortion_coefficients


def undistort_image(img, camera_matrix, distortion_coefficients):
    """
    对单张图像进行去畸变处理。

    :param img: 原始图像（NumPy数组）
    :param camera_matrix: 相机矩阵
    :param distortion_coefficients: 畸变系数
    :return: 去畸变后的图像（NumPy数组）
    """
    # 获取图像尺寸
    h, w = img.shape[:2]

    # 计算去畸变的映射
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 1, (w, h))
    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, distortion_coefficients, None, new_camera_matrix, (w, h),
                                             cv2.CV_16SC2)

    # 应用去畸变
    undistorted_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

    # 可选：裁剪图像以去除边缘黑色区域
    x, y, w, h = roi
    undistorted_img = undistorted_img[y:y + h, x:x + w]

    return undistorted_img


def save_euroc_images(umpackbag, topics, paths, use_message_order=True, fps=30):
    """
    从ROS bag中提取指定话题的图像，并保存到对应的EuRoC格式文件夹中。
    同时根据时间戳生成图像文件名。

    :param umpackbag: 打开的rosbag对象
    :param topics: 包含左、右相机的图像话题名列表
    :param paths: 包含保存左、右相机图像的路径列表
    :param use_message_order: 是否使用消息顺序生成伪时间戳
    :param fps: 伪时间戳的帧率（帧每秒）
    :return: 左、右相机图像的时间戳列表
    """
    bridge = CvBridge()
    # 初始化存储图像名称的字典
    image_names = {topic: [] for topic in topics}
    # 初始化伪时间戳计数器
    counters = {topic: 0 for topic in topics}
    # 计算每帧的时间间隔，单位为纳秒
    frame_interval_ns = int(1e9 / fps)

    for topic, msg, t in umpackbag.read_messages(topics=topics):
        if topic in topics:
            try:
                # 将ROS图像消息转换为OpenCV格式
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print(f"图像转换错误: {e}")
                continue

            if use_message_order:
                # 使用消息的接收顺序生成伪时间戳
                timestamp = counters[topic] * frame_interval_ns
                counters[topic] += 1
            else:
                # 使用消息头部的时间戳
                timestamp = int(msg.header.stamp.to_nsec())

            # 格式化时间戳字符串，确保至少12位，不足前面补零
            image_name = f"{timestamp:012d}.png"
            # 构建图像保存路径
            path = paths[topics.index(topic)]
            image_path = os.path.join(path, image_name)
            # 保存图像
            cv2.imwrite(image_path, cv_image)
            # 记录图像名称
            image_names[topic].append(str(timestamp))
            print(f"保存图像: {image_path}")

    return image_names


def save_euroc_imu(umpackbag, imu_topic, output_file):
    """
    从ROS bag中提取IMU数据，并保存为EuRoC格式的imu0/data.csv文件。

    :param umpackbag: 打开的rosbag对象
    :param imu_topic: IMU数据的话题名
    :param output_file: IMU数据输出文件路径
    """
    with open(output_file, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        # 写入表头
        csvwriter.writerow(['timestamp',
                            'w_RS_S_x', 'w_RS_S_y', 'w_RS_S_z',
                            'a_RS_S_x', 'a_RS_S_y', 'a_RS_S_z'])

        # 遍历IMU消息
        for topic, msg, t in umpackbag.read_messages(topics=[imu_topic]):
            if topic == imu_topic:
                timestamp = int(msg.header.stamp.to_nsec())  # 时间戳，单位纳秒
                # 获取角速度和线性加速度
                wx = msg.angular_velocity.x
                wy = msg.angular_velocity.y
                wz = msg.angular_velocity.z
                ax = msg.linear_acceleration.x
                ay = msg.linear_acceleration.y
                az = msg.linear_acceleration.z
                # 写入CSV文件
                csvwriter.writerow([timestamp, wx, wy, wz, ax, ay, az])
    print(f"IMU数据已保存到 '{output_file}'")


def deal_with_timestamp_euroc(cam0_folder, cam1_folder, output_folder, tolerance=5000):
    """
    处理左右相机的图像时间戳，确保两者图像数量一致，并将右相机的图像复制到与左相机对应的文件夹。
    使用时间戳匹配方法，找到最接近的右相机图像。

    :param cam0_folder: 左相机图像文件夹路径
    :param cam1_folder: 右相机图像文件夹路径
    :param output_folder: 匹配后的右相机图像保存文件夹路径
    :param tolerance: 时间匹配的容差值，单位为微秒
    """
    # 创建匹配后的右相机图像文件夹
    os.makedirs(output_folder, exist_ok=True)
    print(f"创建匹配后右相机图像文件夹: {output_folder}")

    # 获取左右相机图像文件列表，并按文件名排序
    cam0_files = sorted(os.listdir(cam0_folder))
    cam1_files = sorted(os.listdir(cam1_folder))

    # 定义提取文件名中时间戳的函数，假设文件名格式为 "timestamp.png"
    def extract_timestamp(file_name):
        """
        提取文件名中的时间戳部分，假设文件名格式为 "timestamp.png"
        :param file_name: 文件名字符串
        :return: 时间戳整数
        """
        return int(file_name.split('.')[0])

    # 提取左右目文件的时间戳，并构建字典 {timestamp: filename}
    cam0_timestamps = {extract_timestamp(f): f for f in cam0_files}
    cam1_timestamps = {extract_timestamp(f): f for f in cam1_files}

    # 设置一个时间匹配的容差值，单位为微秒
    # tolerance = 5000  # 已作为函数参数

    matched_count = 0  # 记录匹配成功的图像数量
    used_cam1_ts = set()  # 记录已使用的右目时间戳，避免重复匹配

    for ts0, file0 in cam0_timestamps.items():
        # 找到与 ts0 最接近且未被使用的右目图像时间戳
        closest_ts1 = min(cam1_timestamps.keys(), key=lambda ts1: abs(ts1 - ts0))

        # 检查时间差是否在容差范围内，并且右目时间戳未被使用
        if abs(ts0 - closest_ts1) <= tolerance and closest_ts1 not in used_cam1_ts:
            # 构建源文件路径和目标文件路径
            source_path = os.path.join(cam1_folder, cam1_timestamps[closest_ts1])
            target_path = os.path.join(output_folder, file0)
            # 复制文件
            shutil.copyfile(source_path, target_path)
            matched_count += 1
            used_cam1_ts.add(closest_ts1)
            print(f"匹配图像: 左图 {file0} 与 右图 {cam1_timestamps[closest_ts1]}")

    # 输出匹配结果
    print(f"总匹配图像数量: {matched_count}")

    # 检查是否所有左目图像都已匹配
    if matched_count != len(cam0_files):
        print("部分左目图像未能找到匹配的右目图像，请检查时间戳和容差设置。")
    else:
        print("所有左目图像均已成功匹配到右目图像。")

    # 删除原来的右目文件夹，保持目录结构整洁
    if os.path.exists(cam1_folder):
        shutil.rmtree(cam1_folder)
        print(f"删除原右相机图像文件夹: {cam1_folder}")


def main():
    """
    主函数，执行EuRoC格式数据的提取和保存。
    """
    # 设置ROS bag文件路径
    bagname = '/media/asus/32A6EEC5E12B6F66/datasets/6_side_light_2024-10-7-04-08-34.bag'  # 请替换为您的rosbag文件路径

    # 创建EuRoC所需的目录结构
    create_euroc_dirs(base_folder='Dataset')

    # 打开rosbag文件
    try:
        umpackbag = rosbag.Bag(bagname)
        print(f"打开rosbag文件: {bagname}")
    except Exception as e:
        print(f"无法打开rosbag文件: {e}")
        exit(1)

    # 提取并保存IMU数据
    imu_topic = '/lihai/zed_node/imu/data'  # 请替换为您的IMU话题名
    imu_output_file = os.path.join('Dataset', 'imu0', 'data.csv')
    save_euroc_imu(umpackbag, imu_topic, imu_output_file)

    # 提取并保存相机图像
    cam0_topic = '/lihai/zed_node/left/image_rect_color'  # 左相机话题名
    cam1_topic = '/lihai/zed_node/right/image_rect_color'  # 右相机话题名
    topics = [cam0_topic, cam1_topic]
    paths = [os.path.join('Dataset', 'cam0', 'data'),
             os.path.join('Dataset', 'cam1', 'data')]

    # 设置use_message_order为True，使用消息接收顺序生成伪时间戳
    # 如果需要使用实际时间戳，请将use_message_order设置为False
    image_timestamps = save_euroc_images(umpackbag, topics, paths, use_message_order=True, fps=30)

    # 匹配左右相机图像
    cam0_folder = paths[0]
    cam1_folder = paths[1]
    output_folder_path = os.path.join('Dataset', 'cam1', 'data_matched')
    os.makedirs(output_folder_path, exist_ok=True)

    deal_with_timestamp_euroc(cam0_folder, cam1_folder, output_folder_path, tolerance=5000)

    # 关闭rosbag文件
    umpackbag.close()
    print("EuRoC格式数据提取和保存完成")


if __name__ == '__main__':
    main()
