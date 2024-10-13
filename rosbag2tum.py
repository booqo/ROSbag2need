# -*- coding: utf-8 -*-
import rosbag
import csv
from sensor_msgs.msg import Imu
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import shutil
import bisect
# import pandas as pd

# 创建所需的文件夹
def CreateDIR():
    """
    创建保存IMU数据和图像的目录结构。
    主文件夹为 'bag_tum'，包含子文件夹 'left' 和 'right'。
    """
    folder_name = 'bag_tum'
    subfolders = ['left', 'right']

    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
        print(f"创建主文件夹: {folder_name}")

    # 在主文件夹下创建子文件夹
    for subfolder in subfolders:
        subfolder_path = os.path.join(folder_name, subfolder)
        if not os.path.exists(subfolder_path):
            os.makedirs(subfolder_path)
            print(f"创建子文件夹: {subfolder_path}")


# 提取IMU数据并写入CSV文件
def CreateIMUCSV(umpackbag):
    """
    从ROS bag中提取IMU数据，并将其保存到 'imudata.csv' 文件中。

    :param umpackbag: 打开的rosbag对象
    """
    with open('imudata.csv', 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        # 写入表头
        csvwriter.writerow(['timestamp [ns]',
                            'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 'w_RS_S_z [rad s^-1]',
                            'a_RS_S_x [rad m s^-2]', 'a_RS_S_y [rad m s^-2]', 'a_RS_S_z [rad m s^-2]'])

        # 遍历指定话题的消息
        for topic, msg, t in umpackbag.read_messages(topics=['/lihai/zed_node/imu/data']):
            timestamp = msg.header.stamp.to_nsec()  # 时间戳，单位纳秒
            # 获取线性加速度和角速度
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            wx = msg.angular_velocity.x
            wy = msg.angular_velocity.y
            wz = msg.angular_velocity.z
            # 写入CSV文件
            csvwriter.writerow([timestamp, wx, wy, wz, ax, ay, az])
    print("IMU数据已保存到 'imudata.csv'")


# 将IMU CSV文件转换为TXT文件
def TransIMUdatatotxt():
    """
    将 'imudata.csv' 文件转换为 'imudata.txt' 文件，并在表头添加 '#' 符号。
    """
    csv_file = './imudata.csv'
    txt_file = './imudata.txt'

    with open(csv_file, 'r') as file, open(txt_file, 'w') as output_file:
        reader = csv.reader(file)
        writer = csv.writer(output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for i, row in enumerate(reader):
            if i == 0:
                # 表头添加 '#' 符号
                writer.writerow(['#' + cell for cell in row])
            else:
                writer.writerow(row)
    print("IMU数据已转换为 'imudata.txt'")


# 保存相机图像并记录时间戳
def SaveImageFisheye(umpackbag, topics, paths, timestamp_files):
    """
    从ROS bag中提取指定话题的图像，并保存到对应的文件夹，同时记录图像的时间戳。

    :param umpackbag: 打开的rosbag对象
    :param topics: 包含左、右相机的图像话题名列表
    :param paths: 包含保存左、右相机图像的路径列表
    :param timestamp_files: 包含左、右相机时间戳文件路径的列表
    """
    bridge = CvBridge()
    # 初始化存储图像名称的字典
    image_names = {topic: [] for topic in topics}

    for topic, msg, t in umpackbag.read_messages():
        if topic in topics:
            try:
                # 将ROS图像消息转换为OpenCV格式
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print(f"图像转换错误: {e}")
                continue

            # 获取并格式化时间戳
            timestr = "%.9f" % msg.header.stamp.to_sec()
            image_name = timestr.replace('.', '')  # 去掉时间戳中的小数点
            # 构建图像保存路径
            path = paths[topics.index(topic)]
            image_path = os.path.join(path, image_name + '.png')
            # 保存图像
            cv2.imwrite(image_path, cv_image)
            # 记录图像名称
            image_names[topic].append(image_name)
            print(f"保存图像: {image_path}")

    # 将图像名称写入对应的时间戳文件
    for topic, file_path in zip(topics, timestamp_files):
        with open(file_path, 'w') as f:
            f.write('\n'.join(image_names[topic]))
        print(f"保存时间戳文件: {file_path}")


# 处理时间戳，并将左右相机的图像进行匹配
def dealwithTimeStamp():
    """
    处理左右相机的图像时间戳，确保两者图像数量一致，并将右相机的图像复制到与左相机对应的文件夹。
    使用更鲁棒的时间匹配方法，根据时间戳的最接近匹配图像。
    """
    folder1_path = './bag_tum/left/'
    folder2_path = './bag_tum/right/'
    output_folder_path = './bag_tum/right_matched/'

    # 创建匹配后的右相机图像文件夹
    if not os.path.exists(output_folder_path):
        os.makedirs(output_folder_path)
        print(f"创建匹配后右相机图像文件夹: {output_folder_path}")

    # 获取左右相机图像文件列表，并按名称排序
    folder1_files = sorted(os.listdir(folder1_path))
    folder2_files = sorted(os.listdir(folder2_path))

    # 定义提取文件名中时间戳的函数，假设文件名格式为 "timestamp.png"
    def extract_timestamp(file_name):
        """
        提取文件名中的时间戳部分，假设文件名格式为 "timestamp.png"
        :param file_name: 文件名字符串
        :return: 时间戳整数
        """
        return int(file_name.split('.')[0])

    # 提取左右目文件的时间戳，并构建字典 {timestamp: filename}
    folder1_timestamps = {extract_timestamp(f): f for f in folder1_files}
    folder2_timestamps = {extract_timestamp(f): f for f in folder2_files}

    # 设置一个时间匹配的容差值，单位为微秒
    tolerance = 1000  # 可以根据需要调整容差，单位微秒

    matched_count = 0  # 记录匹配成功的图像数量
    used_ts2 = set()  # 记录已使用的右目时间戳，避免重复匹配

    for ts1, file1 in folder1_timestamps.items():
        # 找到与 ts1 最接近且未被使用的右目图像时间戳
        closest_ts2 = min(folder2_timestamps.keys(), key=lambda ts2: abs(ts2 - ts1))

        # 检查时间差是否在容差范围内，并且右目时间戳未被使用
        if abs(ts1 - closest_ts2) <= tolerance and closest_ts2 not in used_ts2:
            # 构建源文件路径和目标文件路径
            source_path = os.path.join(folder2_path, folder2_timestamps[closest_ts2])
            target_path = os.path.join(output_folder_path, file1)
            # 复制文件
            shutil.copyfile(source_path, target_path)
            matched_count += 1
            used_ts2.add(closest_ts2)
            print(f"匹配图像: 左图 {file1} 与 右图 {folder2_timestamps[closest_ts2]}")

    # 输出匹配结果
    print(f"总匹配图像数量: {matched_count}")

    # 检查是否所有左目图像都已匹配
    if matched_count != len(folder1_files):
        print("部分左目图像未能找到匹配的右目图像，请检查时间戳和容差设置。未匹配",len(folder1_files)-matched_count)
    else:
        print("所有左目图像均已成功匹配到右目图像。")

    # 删除原来的右目文件夹，保持目录结构整洁
    if os.path.exists(folder2_path):
        shutil.rmtree(folder2_path)
        print(f"删除原右相机图像文件夹: {folder2_path}")

def dealwithTimeStamp_with_bisect():
    """
    使用bisect模块进行时间戳匹配，确保只输出匹配成功的图像对。
    """
    folder1_path = './bag_tum/left/'
    folder2_path = './bag_tum/right/'
    output_folder_path = './bag_tum/right_matched/'

    # 创建匹配后的右相机图像文件夹
    if not os.path.exists(output_folder_path):
        os.makedirs(output_folder_path)
        print(f"创建匹配后右相机图像文件夹: {output_folder_path}")

    # 获取左右相机图像文件列表，并按名称排序
    folder1_files = sorted(os.listdir(folder1_path))
    folder2_files = sorted(os.listdir(folder2_path))

    # 定义提取文件名中时间戳的函数
    def extract_timestamp(file_name):
        timestamp_str = file_name.split('.')[0].split('_')[0]
        timestamp_sec = float(timestamp_str)
        timestamp_ns = int(timestamp_sec * 1e9)
        return timestamp_ns

    # 提取右相机的时间戳列表，并保持排序
    folder2_timestamps = sorted([extract_timestamp(f) for f in folder2_files])
    folder2_files_sorted = [f for _, f in sorted(zip(folder2_timestamps, folder2_files))]

    matched_count = 0  # 记录匹配成功的图像数量

    for file1 in folder1_files:
        ts1 = extract_timestamp(file1)
        # 使用bisect查找最接近的右相机时间戳
        idx = bisect.bisect_left(folder2_timestamps, ts1)

        closest_ts2 = None
        if idx == 0:
            closest_ts2 = folder2_timestamps[0]
        elif idx == len(folder2_timestamps):
            closest_ts2 = folder2_timestamps[-1]
        else:
            before = folder2_timestamps[idx - 1]
            after = folder2_timestamps[idx]
            if abs(before - ts1) <= abs(after - ts1):
                closest_ts2 = before
            else:
                closest_ts2 = after

        # 检查时间差是否在容差范围内
        tolerance_ns = 1_000_000  # 1 毫秒
        if abs(closest_ts2 - ts1) <= tolerance_ns:
            # 找到匹配的右相机文件
            file2_idx = folder2_timestamps.index(closest_ts2)
            file2 = folder2_files_sorted[file2_idx]
            source_path = os.path.join(folder2_path, file2)
            target_path = os.path.join(output_folder_path, file1)
            shutil.copyfile(source_path, target_path)
            matched_count += 1
            print(f"匹配图像: 左图 {file1} 与 右图 {file2}")

            # 删除已匹配的时间戳和文件，保持同步
            del folder2_timestamps[file2_idx]
            del folder2_files_sorted[file2_idx]
        else:
            print(f"未匹配图像: {file1}")

    # 输出匹配结果
    print(f"总匹配图像数量: {matched_count}")

    # 检查是否所有左目图像都已匹配
    if matched_count != len(folder1_files):
        print(f"部分左目图像未能找到匹配的右目图像，请检查时间戳和容差设置。未匹配数量: {len(folder1_files) - matched_count}")
    else:
        print("所有左目图像均已成功匹配到右目图像。")

    # 删除原来的右目文件夹，保持目录结构整洁
    if os.path.exists(folder2_path):
        shutil.rmtree(folder2_path)
        print(f"删除原右相机图像文件夹: {folder2_path}")

# def dealwithTimeStamp_with_pandas():
#     """
#     使用pandas进行时间戳匹配，确保只输出匹配成功的图像对。
#     """
#     folder1_path = './bag_tum/left/'
#     folder2_path = './bag_tum/right/'
#     output_folder_path = './bag_tum/right_matched/'
#
#     # 创建匹配后的右相机图像文件夹
#     if not os.path.exists(output_folder_path):
#         os.makedirs(output_folder_path)
#         print(f"创建匹配后右相机图像文件夹: {output_folder_path}")
#
#     # 获取左右相机图像文件列表，并按名称排序
#     folder1_files = sorted(os.listdir(folder1_path))
#     folder2_files = sorted(os.listdir(folder2_path))
#
#     # 定义提取文件名中时间戳的函数
#     def extract_timestamp(file_name):
#         timestamp_str = file_name.split('.')[0].split('_')[0]
#         timestamp_sec = float(timestamp_str)
#         timestamp_ns = int(timestamp_sec * 1e9)
#         return timestamp_ns
#
#     # 创建DataFrame
#     df_left = pd.DataFrame({
#         'filename': folder1_files,
#         'timestamp_ns': [extract_timestamp(f) for f in folder1_files]
#     })
#
#     df_right = pd.DataFrame({
#         'filename': folder2_files,
#         'timestamp_ns': [extract_timestamp(f) for f in folder2_files]
#     })
#
#     # 设置容差
#     tolerance_ns = 1_000_000  # 1 毫秒
#
#     # 使用merge_asof进行匹配
#     df_right_sorted = df_right.sort_values('timestamp_ns')
#     df_left_sorted = df_left.sort_values('timestamp_ns')
#
#     merged = pd.merge_asof(df_left_sorted, df_right_sorted, on='timestamp_ns', direction='nearest', tolerance=tolerance_ns, suffixes=('_left', '_right'))
#
#     # 过滤掉未匹配的行
#     matched = merged.dropna(subset=['filename_right'])
#
#     # 复制匹配的图像
#     for _, row in matched.iterrows():
#         source_path = os.path.join(folder2_path, row['filename_right'])
#         target_path = os.path.join(output_folder_path, row['filename_left'])
#         shutil.copyfile(source_path, target_path)
#         print(f"匹配图像: 左图 {row['filename_left']} 与 右图 {row['filename_right']}")
#
#     matched_count = len(matched)
#     print(f"总匹配图像数量: {matched_count}")
#
#     # 检查是否所有左目图像都已匹配
#     if matched_count != len(folder1_files):
#         print(f"部分左目图像未能找到匹配的右目图像，请检查时间戳和容差设置。未匹配数量: {len(folder1_files) - matched_count}")
#     else:
#         print("所有左目图像均已成功匹配到右目图像。")
#
#     # 删除原来的右目文件夹，保持目录结构整洁
#     if os.path.exists(folder2_path):
#         shutil.rmtree(folder2_path)
#         print(f"删除原右相机图像文件夹: {folder2_path}")


# 主程序入口

if __name__ == '__main__':
    # 指定要处理的rosbag文件路径
    bagname = '/media/asus/32A6EEC5E12B6F66/datasets/6_side_light_2024-10-7-04-08-34.bag'

    # 打开rosbag文件
    try:
        umpackbag = rosbag.Bag(bagname)
        print(f"打开rosbag文件: {bagname}")
    except Exception as e:
        print(f"无法打开rosbag文件: {e}")
        exit(1)

    # 创建保存数据的目录结构
    CreateDIR()

    # 提取IMU数据并保存
    CreateIMUCSV(umpackbag)
    TransIMUdatatotxt()

    # 提取左、右相机图像并保存
    topics = ["/lihai/zed_node/left/image_rect_color", "/lihai/zed_node/right/image_rect_color"]
    paths = ['./bag_tum/left/', './bag_tum/right/']
    timestamp_files = ['./bag_tum/left_timestamp.txt', './bag_tum/right_timestamp.txt']
    SaveImageFisheye(umpackbag, topics, paths, timestamp_files)

    # 处理时间戳，匹配左右相机图像
    dealwithTimeStamp_with_bisect()

    # 关闭rosbag文件
    umpackbag.close()
    print("处理完成")
