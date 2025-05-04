from enum import Enum
import os
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import UInt8


class DetectSign(Node):

    def __init__(self):
        super().__init__('detect_apriltag_image')

        self.sub_image_type = 'raw'  # or 'compressed'
        self.pub_image_type = 'compressed'  # or 'raw'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage,
                '/detect/image_input/compressed',
                self.cbFindAprilTagImage,
                10
            )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image,
                '/detect/image_input',
                self.cbFindAprilTagImage,
                10
            )

        self.pub_traffic_sign = self.create_publisher(UInt8, '/detect/traffic_sign', 10)
        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign = self.create_publisher(
                CompressedImage,
                '/detect/image_output/compressed', 10
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_traffic_sign = self.create_publisher(
                Image, '/detect/image_output', 10
            )

        self.cvBridge = CvBridge()
        self.TrafficSign = Enum('TrafficSign', 'apriltag')
        self.counter = 1

        self.fnPreproc()

        self.get_logger().info('DetectSign Node Initialized for AprilTag Image')

    def fnPreproc(self):
        self.sift = cv2.SIFT_create()

        # Path to your AprilTag image (used in Gazebo)
        apriltag_img_path = os.path.expanduser(
            '~/.gazebo/models/Apriltag36_11_00001/materials/textures/tag36_11_00001.png'
        )

        self.img_apriltag = cv2.imread(apriltag_img_path, 0)
        if self.img_apriltag is None:
            self.get_logger().error(f"Failed to load AprilTag image from {apriltag_img_path}")
            return

        self.kp_apriltag, self.des_apriltag = self.sift.detectAndCompute(
            self.img_apriltag, None
        )

        FLANN_INDEX_KDTREE = 0
        index_params = {'algorithm': FLANN_INDEX_KDTREE, 'trees': 5}
        search_params = {'checks': 50}
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def fnCalcMSE(self, arr1, arr2):
        squared_diff = (arr1 - arr2) ** 2
        total_sum = np.sum(squared_diff)
        num_all = arr1.shape[0] * arr1.shape[1]
        err = total_sum / num_all
        return err

    def cbFindAprilTagImage(self, image_msg):
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == 'raw':
            cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        MIN_MATCH_COUNT = 8
        MIN_MSE_DECISION = 50000

        kp1, des1 = self.sift.detectAndCompute(cv_image_input, None)
        if des1 is None or self.des_apriltag is None:
            return

        matches_apriltag = self.flann.knnMatch(des1, self.des_apriltag, k=2)

        good_apriltag = []
        for m, n in matches_apriltag:
            if m.distance < 0.7 * n.distance:
                good_apriltag.append(m)

        if len(good_apriltag) > MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_apriltag]).reshape(-1, 1, 2)
            dst_pts = np.float32([self.kp_apriltag[m.trainIdx].pt for m in good_apriltag]).reshape(-1, 1, 2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            matches_mask = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.apriltag.value
                self.pub_traffic_sign.publish(msg_sign)
                self.get_logger().info('AprilTag image detected')

                final_img = cv2.drawMatches(
                    cv_image_input, kp1, self.img_apriltag, self.kp_apriltag,
                    good_apriltag, None,
                    matchColor=(0, 255, 0),
                    singlePointColor=None,
                    matchesMask=matches_mask,
                    flags=2
                )

                if self.pub_image_type == 'compressed':
                    self.pub_image_traffic_sign.publish(
                        self.cvBridge.cv2_to_compressed_imgmsg(final_img, 'jpg')
                    )
                else:
                    self.pub_image_traffic_sign.publish(
                        self.cvBridge.cv2_to_imgmsg(final_img, 'bgr8')
                    )
                return

        # If no detection
        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(cv_image_input, 'jpg')
            )
        else:
            self.pub_image_traffic_sign.publish(
                self.cvBridge.cv2_to_imgmsg(cv_image_input, 'bgr8')
            )


def main(args=None):
    rclpy.init(args=args)
    node = DetectSign()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
