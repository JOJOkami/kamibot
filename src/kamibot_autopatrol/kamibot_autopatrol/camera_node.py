import math
import rclpy  # ROS2 Python接口库
from rclpy.node import Node  # ROS2 节点类
import tf_transformations  # TF坐标变换库
from tf2_ros import TransformException  # TF左边变换的异常类
from tf2_ros.buffer import Buffer  # 存储坐标变换信息的缓冲类
from tf2_ros.transform_listener import TransformListener  # 监听坐标变换的监听器类
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class CameraNode(Node):

    def __init__(self, name="camera_node"):
        super().__init__(name)
        # 获取保存路径
        self.declare_parameter("image_save_path", "")
        self.image_save_path = self.get_parameter("image_save_path").value

        self.declare_parameter("source_frame", "map")
        self.source_frame = (
            self.get_parameter("source_frame").get_parameter_value().string_value
        )

        self.declare_parameter("target_frame", "base_footprint")
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )

        self.tf_buffer = Buffer()  # 创建保存坐标变换信息的缓冲区
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )  # 创建坐标变换的监听器

        self.bridge = CvBridge()
        self.latest_image = None
        self.subscription_image = self.create_subscription(
            Image, "/kamibot/image_raw", self.record_image, 10
        )

        self.pos = None
        self.euler = None
        self.passed_points = []

    def get_tf(self):
        try:
            now = rclpy.time.Time()
            # 监听当前时刻源坐标系到目标坐标系的坐标变换
            trans = self.tf_buffer.lookup_transform(
                self.source_frame, self.target_frame, now
            )
        except TransformException as ex:  # 如果坐标变换获取失败，进入异常报告
            self.get_logger().info(
                f"Could not transform {self.target_frame} to {self.source_frame}: {ex}"
            )
            return

        self.pos = trans.transform.translation  # 获取位置信息
        quat = trans.transform.rotation  # 获取姿态信息（四元数）
        self.euler = tf_transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w]
        )
        self.get_logger().info(
            f"机器人坐标为[{self.pos.x}, {self.pos.y}, {self.pos.z}]"
        )

    def has_be_recored(self, target_pos, radius=0.4):
        target_x, target_y = target_pos
        for point in self.passed_points:
            x, y = point
            if (x - radius < target_x < x + radius
                and y - radius < target_y < y + radius):
                return True
        return False

    def has_red_dot(self, image, area_threshold=0.02):
        # 转换为 HSV 颜色空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 定义红色的 HSV 范围（低和高）
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # 创建红色掩膜
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # 形态学操作（可选：去除噪声）
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        # 计算红色像素数量
        red_pixel_count = cv2.countNonZero(red_mask)

        # 计算图像总像素数
        total_pixels = image.shape[0] * image.shape[1]

        # 计算红色区域占比
        red_area_ratio = red_pixel_count / total_pixels

        # 判断红色区域面积是否超过阈值
        if red_area_ratio > area_threshold:
            return True
        else:
            return False

    def compute_front_position(self, distance=0.4):
        """
        计算机器人正前方指定距离的坐标。
        """
        yaw = self.euler[2]
        x_new = self.pos.x + distance * math.cos(yaw)
        y_new = self.pos.y + distance * math.sin(yaw)
        return (x_new, y_new)

    def record_image(self, data):
        """
        判断路径上是否有红点并记录
        """
        # pose = self.get_current_pose()
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        if self.has_red_dot(cv_image):
            self.get_logger().info("存在红点")
            self.get_tf()

            if self.pos is None or self.euler is None:
                return
            target_pos = self.compute_front_position()
            if self.has_be_recored(target_pos):
                return

            self.get_logger().info(f"记录红点的坐标: {target_pos}")
            self.passed_points.append(target_pos)

            image_name = f"image_{target_pos[0]:3.2f}_{target_pos[1]:3.2f}.png"
            self.get_logger().info(f"记录打卡点图片: {image_name}")
            cv2.imwrite(f"{self.image_save_path}/{image_name}", cv_image)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode("camera_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
