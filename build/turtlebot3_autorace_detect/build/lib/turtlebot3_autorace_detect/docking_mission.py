from enum import Enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import UInt8, Int32, Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector


class DetectSign(Node):

    def __init__(self):
        super().__init__('detect_apriltag_image')

        self.sub_image_type = 'raw'
        self.pub_image_type = 'compressed'

        # Subscriptions and Publications
        self.sub_image_original = self.create_subscription(
            Image if self.sub_image_type == 'raw' else CompressedImage,
            '/camera/image_raw',
            self.cbFindAprilTagImage,
            10
        )

        self.pub_traffic_sign = self.create_publisher(UInt8, '/detect/traffic_sign', 10)
        self.pub_image_traffic_sign = self.create_publisher(
            CompressedImage if self.pub_image_type == 'compressed' else Image,
            '/detect/image_output/compressed' if self.pub_image_type == 'compressed' else '/detect/image_output',
            10
        )
        self.pub_center_x = self.create_publisher(Int32, '/apriltag/center_x', 10)
        self.pub_distance = self.create_publisher(Float32, '/apriltag/distance', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cvBridge = CvBridge()
        self.detector = Detector(families='tag36h11')

        self.TrafficSign = Enum('TrafficSign', 'apriltag')
        self.counter = 1
        self.get_logger().info('AprilTag Detector initialized')

    def cbFindAprilTagImage(self, image_msg):
        if self.counter % 3 != 0:
            self.counter += 1
            return
        self.counter = 1

        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            frame = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        if detections:
            self.get_logger().info(f"Detected {len(detections)} AprilTag(s)")

            msg_sign = UInt8()
            msg_sign.data = self.TrafficSign.apriltag.value
            self.pub_traffic_sign.publish(msg_sign)

            for det in detections:
                corners = det.corners.astype(int)
                cx, cy = int(det.center[0]), int(det.center[1])

                msg_cx = Int32()
                msg_cx.data = cx
                self.pub_center_x.publish(msg_cx)

                tag_size_px = np.linalg.norm(det.corners[0] - det.corners[2])
                distance_est = 2000.0 / tag_size_px
                msg_dist = Float32()
                msg_dist.data = float(distance_est)
                self.pub_distance.publish(msg_dist)

                cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
                cv2.putText(frame, f"ID:{det.tag_id} Dist:{distance_est:.2f}", (cx, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # --- Control robot motion ---
                image_center = 160
                error_x = cx - image_center

                cmd = Twist()
                kp_linear = 0.005
                kp_angular = 0.005
                angle_threshold = 15
                stop_distance = 13.0  # Stop when within 13 cm

                if distance_est <= stop_distance:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.get_logger().info("Stopping near tag")
                elif abs(error_x) > angle_threshold:
                    cmd.linear.x = 0.0
                    cmd.angular.z = -kp_angular * error_x
                    self.get_logger().info(f"Turning: angular.z = {cmd.angular.z:.2f}")
                else:
                    cmd.linear.x = kp_linear * distance_est
                    cmd.angular.z = 0.0
                    self.get_logger().info(f"Forward: linear.x = {cmd.linear.x:.2f}")

                self.cmd_vel_pub.publish(cmd)

        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(frame, 'jpg')
            )
        else:
            self.pub_image_traffic_sign.publish(
                self.cvBridge.cv2_to_imgmsg(frame, 'bgr8')
            )


def main(args=None):
    rclpy.init(args=args)
    node = DetectSign()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

