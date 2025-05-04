import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class AprilTagDocking(Node):
    def __init__(self):
        super().__init__('apriltag_docking_node')
        self.bridge = CvBridge()

        self.subscription_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.subscription_scan = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.distance_pub = self.create_publisher(Float32, '/tag_distance', 10)

        self.latest_distance = None
        self.found_tag = False
        self.center_x = 0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('AprilTag Docking Node Started')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        detector = cv2.AprilTagDetector()
        detector_options = cv2.AprilTagDetector_Params()
        tags = detector.detect(gray)

        if tags:
            self.found_tag = True
            tag = tags[0]  # Assume one tag
            self.center_x = int(tag.center[0])
        else:
            self.found_tag = False

    def lidar_callback(self, msg):
        # Take a small front range slice to estimate distance
        front_ranges = msg.ranges[len(msg.ranges)//2 - 5: len(msg.ranges)//2 + 5]
        front_ranges = [r for r in front_ranges if not np.isnan(r) and not np.isinf(r)]
        if front_ranges:
            self.latest_distance = min(front_ranges)
            self.distance_pub.publish(Float32(data=self.latest_distance))

    def control_loop(self):
        if not self.found_tag or self.latest_distance is None:
            return

        cmd = Twist()

        image_center = 320  # Assuming 640x480 resolution
        error_x = self.center_x - image_center

        if abs(error_x) > 20:
            cmd.angular.z = -0.002 * error_x  # Turn toward tag
        else:
            if self.latest_distance > 0.4:
                cmd.linear.x = 0.1  # Move forward
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDocking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
