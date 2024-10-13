# -*- coding: utf-8 -*-
import rosbag
import csv
from sensor_msgs.msg import Imu
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import shutil


# 创建所需的文件夹
def CreateDIR():
    folder_name = 'bag_tum'
    subfolders = ['left', 'right']
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    # 在主文件夹下创建子文件夹
    for subfolder in subfolders:
        subfolder_path = os.path.join(folder_name, subfolder)
        if not os.path.exists(subfolder_path):
            os.makedirs(subfolder_path)


# 提取IMU数据并写入CSV文件
def CreateIMUCSV(umpackbag):
    csvfile = open('imudata.csv', 'w')
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 'w_RS_S_z [rad s^-1]',
                        'a_RS_S_x [rad m s^-2]', 'a_RS_S_y [rad m s^-2]', 'a_RS_S_z [rad m s^-2]'])
    for topic, msg, t in umpackbag.read_messages(topics=['/lihai/zed_node/imu/data']):
        timestamp = msg.header.stamp.to_nsec()
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
        csvwriter.writerow([timestamp, wx, wy, wz, ax, ay, az])
    csvfile.close()


# 将IMU CSV文件转换为TXT文件
def TransIMUdatatotxt():
    csv_file = './imudata.csv'
    txt_file = './imudata.txt'
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        with open(txt_file, 'w') as output_file:
            writer = csv.writer(output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for i, row in enumerate(reader):
                if i == 0:
                    writer.writerow(['#' + cell for cell in row])  # 添加#号
                else:
                    writer.writerow(row)


# 提取左相机的图像并保存
def SaveImageFishereyeleft(umpackbag, bagname):
    path = './bag_tum/left/'
    txt_file = './timestamp.txt'
    bridge = CvBridge()
    image_names = []

    for topic, msg, t in umpackbag.read_messages():
        if topic == "/lihai/zed_node/left/image_rect_color":
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print(e)
                continue
            timestr = "%.9f" % msg.header.stamp.to_sec()
            image_name = timestr.replace('.', '')  # 去掉时间戳中的小数点
            cv2.imwrite(path + image_name + '.png', cv_image)  # 以PNG格式保存
            image_names.append(image_name)  # 将图像名称添加到列表中

    with open(txt_file, 'w') as f:
        f.write('\n'.join(image_names))


# 提取右相机的图像并保存
def SaveImageFishereyeright(umpackbag, bagname):
    path = './bag_tum/right/'
    bridge = CvBridge()
    image_names = []

    for topic, msg, t in umpackbag.read_messages():
        if topic == "/lihai/zed_node/right/image_rect_color":
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print(e)
                continue
            timestr = "%.9f" % msg.header.stamp.to_sec()
            image_name = timestr.replace('.', '')  # 去掉时间戳中的小数点
            cv2.imwrite(path + image_name + '.png', cv_image)  # 以PNG格式保存
            image_names.append(image_name)  # 将图像名称添加到列表中

def SaveImageFisheye(umpackbag, topics, paths, timestamp_files):
    """
    优化后的函数，用于提取左、右相机的图像并保存。

    :param umpackbag: rosbag 包
    :param topics: 包含左、右相机的图像话题名列表
    :param paths: 包含保存左、右相机图像的路径列表
    :param timestamp_files: 包含左、右相机时间戳文件路径的列表
    """
    bridge = CvBridge()
    image_names = {topic: [] for topic in topics}  # 创建字典来存储不同相机的话题名称和对应的图像名称

    for topic, msg, t in umpackbag.read_messages():
        if topic in topics:
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print(e)
                continue

            timestr = "%.9f" % msg.header.stamp.to_sec()
            image_name = timestr.replace('.', '')  # 去掉时间戳中的小数点
            path = paths[topics.index(topic)]  # 获取相应的路径
            cv2.imwrite(path + image_name + '.png', cv_image)  # 以PNG格式保存
            image_names[topic].append(image_name)  # 将图像名称添加到相应列表中

    # 写入时间戳文件
    for topic, file_path in zip(topics, timestamp_files):
        with open(file_path, 'w') as f:
            f.write('\n'.join(image_names[topic]))


# 处理时间戳，并将左右相机的图像进行匹配
def dealwithTimeStamp():
    folder1_path = './bag_tum/left/'
    folder2_path = './bag_tum/right/'
    output_folder_path = './bag_tum/right_matched/'

    if not os.path.exists(output_folder_path):
        os.makedirs(output_folder_path)

    folder1_files = sorted(os.listdir(folder1_path))
    folder2_files = sorted(os.listdir(folder2_path))

    if len(folder1_files) != len(folder2_files):
        print("录制包时左右目图像数量不一致，请手动处理")
    else:
        for i in range(len(folder2_files)):
            source_path = os.path.join(folder2_path, folder2_files[i])
            target_path = os.path.join(output_folder_path, folder1_files[i])
            shutil.copyfile(source_path, target_path)
        print("图像序列生成完毕")

    if os.path.exists(folder2_path):
        shutil.rmtree(folder2_path)  # 删除原来的右目文件夹


# 主程序入口
if __name__ == '__main__':
    bagname = '/media/asus/32A6EEC5E12B6F66/datasets/6_side_light_2024-10-7-04-08-34.bag'
    umpackbag = rosbag.Bag(bagname)
    CreateDIR()
    CreateIMUCSV(umpackbag)
    TransIMUdatatotxt()
    SaveImageFishereyeleft(umpackbag, bagname)
    SaveImageFishereyeright(umpackbag, bagname)
    topics = ["/lihai/zed_node/left/image_rect_color", "/lihai/zed_node/right/image_rect_color"]
    paths = ['./bag_tum/left/', './bag_tum/right/']
    timestamp_files = ['./left_timestamp.txt', './right_timestamp.txt']
    SaveImageFisheye(umpackbag, topics, paths, timestamp_files)
    dealwithTimeStamp()