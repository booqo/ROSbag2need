# -*- coding: utf-8 -*-
import cv2
import os
import yaml
import numpy as np


def opencv_matrix_constructor(loader, node):
    """
    自定义解析OpenCV格式的矩阵类型。
    """
    mapping = loader.construct_mapping(node, deep=True)
    data = mapping['data']
    rows = mapping['rows']
    cols = mapping['cols']

    # 将OpenCV矩阵数据转换为NumPy数组
    return np.array(data).reshape((rows, cols))


# 注册OpenCV矩阵类型处理器
yaml.add_constructor('tag:yaml.org,2002:opencv-matrix', opencv_matrix_constructor)


def load_camera_parameters(calibration_file, camera_side):
    """
    从YAML格式的校准文件中加载指定相机的内参和畸变系数。

    :param calibration_file: 校准文件的路径
    :param camera_side: 'LEFT' 或 'RIGHT'，指定加载哪个相机的参数
    :return: 相机矩阵和畸变系数
    """
    with open(calibration_file, 'r') as file:
        calib_data = yaml.safe_load(file)

    # 根据左右相机加载不同的内参和畸变系数
    camera_matrix = np.array(calib_data[f'{camera_side}.K']['data']).reshape(3, 3).astype(np.float32)
    distortion_coefficients = np.array(calib_data[f'{camera_side}.D']['data']).astype(np.float32)

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
    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, distortion_coefficients, None, new_camera_matrix, (w, h), 5)

    # 应用去畸变
    undistorted_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

    # 可选：裁剪图像以去除边缘黑色区域
    x, y, w, h = roi
    undistorted_img = undistorted_img[y:y + h, x:x + w]

    return undistorted_img


def undistort_images_in_folder(input_folder, output_folder, camera_matrix, distortion_coefficients):
    """
    对指定文件夹中的所有图像进行去畸变处理，并保存到输出文件夹中。

    :param input_folder: 输入图像文件夹路径
    :param output_folder: 输出图像文件夹路径
    :param camera_matrix: 相机矩阵
    :param distortion_coefficients: 畸变系数
    """
    # 创建输出文件夹（如果不存在）
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        print(f"创建输出文件夹: {output_folder}")

    # 遍历输入文件夹中的所有文件
    for filename in os.listdir(input_folder):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff')):
            input_path = os.path.join(input_folder, filename)
            output_path = os.path.join(output_folder, filename)

            # 读取图像
            img = cv2.imread(input_path)
            if img is None:
                print(f"无法读取图像: {input_path}")
                continue

            # 去畸变
            undistorted_img = undistort_image(img, camera_matrix, distortion_coefficients)

            # 保存去畸变后的图像
            cv2.imwrite(output_path, undistorted_img)
            print(f"保存去畸变后的图像: {output_path}")


def main():
    """
    主函数，执行去畸变处理。
    """
    # 设置相机校准文件路径
    calibration_file = '/media/asus/32A6EEC5E12B6F66/ORB_SLAM3_/Examples/Stereo/zedm（复件）.yaml'  # 请替换为您的校准文件路径

    # 设置输入和输出文件夹路径
    input_left_folder = './bag_tum/left/'
    input_right_folder = './bag_tum/right_matched/'  # 使用匹配后的右相机图像
    output_left_folder = './bag_tum/left_undistorted/'
    output_right_folder = './bag_tum/right_undistorted/'

    # 分别加载左相机和右相机的内参和畸变系数
    left_camera_matrix, left_distortion_coefficients = load_camera_parameters(calibration_file, 'LEFT')
    right_camera_matrix, right_distortion_coefficients = load_camera_parameters(calibration_file, 'RIGHT')
    print("左右相机的内参和畸变系数已加载")

    # 对左相机图像进行去畸变
    print("开始去畸变左相机图像...")
    undistort_images_in_folder(input_left_folder, output_left_folder, left_camera_matrix, left_distortion_coefficients)

    # 对右相机图像进行去畸变
    print("开始去畸变右相机图像...")
    undistort_images_in_folder(input_right_folder, output_right_folder, right_camera_matrix,
                               right_distortion_coefficients)

    print("所有图像去畸变处理完成")


if __name__ == '__main__':
    main()
