import time

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image

from uav_vln_interfaces.msg import TargetDetection


class RedTargetDetector(Node):

    def __init__(self):

        super().__init__('red_target_detector')

        self.declare_parameter(
            'image_topic',
            '/uav/camera/color/image_raw',
        )

        self.declare_parameter(
            'min_area_ratio',
            0.002,
        )

        self.declare_parameter(
            'min_confidence',
            0.20,
        )

        self.image_topic = (
            self.get_parameter('image_topic')
            .get_parameter_value()
            .string_value
        )

        self.min_area_ratio = (
            self.get_parameter('min_area_ratio')
            .get_parameter_value()
            .double_value
        )

        self.min_confidence = (
            self.get_parameter('min_confidence')
            .get_parameter_value()
            .double_value
        )

        self.bridge = CvBridge()

        self.detection_pub = self.create_publisher(
            TargetDetection,
            '/uav/perception/target_detection',
            10,
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/uav/perception/debug_image',
            10,
        )

        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Red target detector started. '
            f'Image topic: {self.image_topic}'
        )

    def image_callback(self, msg: Image):

        try:

            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8',
            )

        except Exception as exc:

            self.get_logger().error(
                f'cv_bridge conversion failed: {exc}'
            )

            return

        height, width = image.shape[:2]

        hsv = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2HSV,
        )

        # Red wraps around the HSV hue boundary.
        lower_red_1 = np.array(
            [0, 100, 70],
            dtype=np.uint8,
        )

        upper_red_1 = np.array(
            [10, 255, 255],
            dtype=np.uint8,
        )

        lower_red_2 = np.array(
            [170, 100, 70],
            dtype=np.uint8,
        )

        upper_red_2 = np.array(
            [179, 255, 255],
            dtype=np.uint8,
        )

        mask_1 = cv2.inRange(
            hsv,
            lower_red_1,
            upper_red_1,
        )

        mask_2 = cv2.inRange(
            hsv,
            lower_red_2,
            upper_red_2,
        )

        mask = cv2.bitwise_or(
            mask_1,
            mask_2,
        )

        kernel = np.ones(
            (5, 5),
            np.uint8,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel,
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        detection = TargetDetection()

        detection.timestamp_us = (
            time.time_ns() // 1000
        )

        detection.detected = False
        detection.label = 'red_house'
        detection.confidence = 0.0

        debug = image.copy()

        if contours:

            contour = max(
                contours,
                key=cv2.contourArea,
            )

            area = float(
                cv2.contourArea(contour)
            )

            image_area = float(
                width * height
            )

            area_ratio = (
                area / image_area
                if image_area > 0.0
                else 0.0
            )

            if area_ratio >= self.min_area_ratio:

                x, y, w, h = cv2.boundingRect(
                    contour
                )

                center_u = (
                    x + 0.5 * w
                )

                center_v = (
                    y + 0.5 * h
                )

                # A deliberately simple confidence model.
                confidence = min(
                    1.0,
                    area_ratio / 0.05,
                )

                if confidence >= self.min_confidence:

                    detection.detected = True

                    detection.confidence = float(
                        confidence
                    )

                    detection.center_u_norm = float(
                        center_u / width
                    )

                    detection.center_v_norm = float(
                        center_v / height
                    )

                    detection.bbox_xmin_norm = float(
                        x / width
                    )

                    detection.bbox_ymin_norm = float(
                        y / height
                    )

                    detection.bbox_xmax_norm = float(
                        (x + w) / width
                    )

                    detection.bbox_ymax_norm = float(
                        (y + h) / height
                    )

                    detection.area_ratio = float(
                        area_ratio
                    )

                    cv2.rectangle(
                        debug,
                        (x, y),
                        (x + w, y + h),
                        (0, 255, 0),
                        2,
                    )

                    cv2.circle(
                        debug,
                        (
                            int(center_u),
                            int(center_v),
                        ),
                        5,
                        (255, 0, 0),
                        -1,
                    )

                    cv2.putText(
                        debug,
                        (
                            f'red_house '
                            f'{confidence:.2f}'
                        ),
                        (x, max(20, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )

        self.detection_pub.publish(
            detection
        )

        try:

            debug_msg = (
                self.bridge.cv2_to_imgmsg(
                    debug,
                    encoding='bgr8',
                )
            )

            debug_msg.header = msg.header

            self.debug_pub.publish(
                debug_msg
            )

        except Exception as exc:

            self.get_logger().warning(
                f'Debug image publish failed: {exc}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = RedTargetDetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
