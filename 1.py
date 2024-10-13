import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# 路径设置
rgb_path = '/home/asus/PycharmProjects/ll_seathru_splatting/output'  # RGB 彩色图像存储目录
depth_path = '/home/asus/PycharmProjects/ll_seathru_splatting/output'  # 深度图像存储目录

class ImageCreator():
    def __init__(self):
        self.bridge = CvBridge()

        # 确保存储目录存在
        if not os.path.exists(rgb_path):
            os.makedirs(rgb_path)

        if not os.path.exists(depth_path):
            os.makedirs(depth_path)

        # 读取 .bag 文件
        with rosbag.Bag('/media/asus/E/历史数据/15小水池不同光照数据集/datasets/1_top_light_2024-10-7-02-11-21.bag', 'r') as bag:  # 替换为实际的 .bag 文件路径
            for topic, msg, t in bag.read_messages():
                if topic == "/lihai/zed_node/left/image_rect_color":  # RGB 图像的 topic
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    except CvBridgeError as e:
                        print(f"Error converting RGB image: {e}")
                        continue
                    # 保存 RGB 图像
                    timestr = "%.6f" % msg.header.stamp.to_sec()  # 使用时间戳作为文件名
                    image_name = timestr + ".png"
                    cv2.imwrite(os.path.join(rgb_path, image_name), cv_image)
                    print(f"Saved RGB image: {image_name}")

                elif topic == "/lihai/zed_node/depth/depth_registered":  # 深度图像的 topic
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                    except CvBridgeError as e:
                        print(f"Error converting depth image: {e}")
                        continue
                    # 保存深度图像
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    image_name = timestr + ".png"
                    cv2.imwrite(os.path.join(depth_path, image_name), cv_image)
                    print(f"Saved depth image: {image_name}")

if __name__ == '__main__':
    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
