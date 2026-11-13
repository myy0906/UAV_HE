import math
import time
from collections import deque

import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image

from uav_vln_interfaces.msg import (
    SemanticTarget,
    TargetDetection,
    VehicleState,
)


class TargetMapper(Node):

    def __init__(self):

        super().__init__('target_mapper')

        # --------------------------------------------------------
        # PX4 x500_depth / OAK-D mount relative to base_link.
        #
        # Forward / Left / Up body convention:
        # x = forward
        # y = left
        # z = up
        #
        # Values are parameters so that they can be calibrated later.
        # --------------------------------------------------------

        self.declare_parameter(
            'camera_mount_x',
            0.12,
        )

        self.declare_parameter(
            'camera_mount_y',
            0.03,
        )

        self.declare_parameter(
            'camera_mount_z',
            0.242,
        )

        self.declare_parameter(
            'depth_patch_radius',
            4,
        )

        self.declare_parameter(
            'min_depth_m',
            0.3,
        )

        self.declare_parameter(
            'max_depth_m',
            40.0,
        )

        self.declare_parameter(
            'stable_min_samples',
            5,
        )

        self.declare_parameter(
            'stable_scatter_xy_m',
            0.75,
        )

        self.mount_x = (
            self.get_parameter('camera_mount_x')
            .value
        )

        self.mount_y = (
            self.get_parameter('camera_mount_y')
            .value
        )

        self.mount_z = (
            self.get_parameter('camera_mount_z')
            .value
        )

        self.patch_radius = int(
            self.get_parameter(
                'depth_patch_radius'
            ).value
        )

        self.min_depth = float(
            self.get_parameter(
                'min_depth_m'
            ).value
        )

        self.max_depth = float(
            self.get_parameter(
                'max_depth_m'
            ).value
        )

        self.stable_min_samples = int(
            self.get_parameter(
                'stable_min_samples'
            ).value
        )

        self.stable_scatter_xy = float(
            self.get_parameter(
                'stable_scatter_xy_m'
            ).value
        )

        self.bridge = CvBridge()

        self.latest_depth = None
        self.latest_depth_time = 0.0

        self.depth_info = None

        self.vehicle_state = None

        self.history = deque(
            maxlen=15
        )

        self.target_pub = self.create_publisher(
            SemanticTarget,
            '/uav/mapping/semantic_target',
            10,
        )

        self.create_subscription(
            Image,
            '/uav/camera/depth/image_raw',
            self.depth_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            CameraInfo,
            '/uav/camera/depth/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            VehicleState,
            '/uav/vehicle_state',
            self.vehicle_state_callback,
            10,
        )

        self.create_subscription(
            TargetDetection,
            '/uav/perception/target_detection',
            self.detection_callback,
            10,
        )

        self.get_logger().info(
            'Target mapper started.'
        )

    # ============================================================
    # Inputs
    # ============================================================

    def depth_callback(
        self,
        msg: Image,
    ):

        try:

            depth = (
                self.bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding='passthrough',
                )
            )

            depth = np.asarray(
                depth
            )

            # 16UC1 is normally millimetres.
            if depth.dtype == np.uint16:

                depth = (
                    depth.astype(np.float32)
                    / 1000.0
                )

            else:

                depth = depth.astype(
                    np.float32
                )

            self.latest_depth = depth

            self.latest_depth_time = (
                time.monotonic()
            )

        except Exception as exc:

            self.get_logger().error(
                f'Depth conversion failed: {exc}'
            )

    def camera_info_callback(
        self,
        msg: CameraInfo,
    ):

        self.depth_info = msg

    def vehicle_state_callback(
        self,
        msg: VehicleState,
    ):

        self.vehicle_state = msg

    # ============================================================
    # Detection -> 3D target
    # ============================================================

    def detection_callback(
        self,
        detection: TargetDetection,
    ):

        if not detection.detected:

            return

        if self.latest_depth is None:

            return

        if self.depth_info is None:

            return

        if self.vehicle_state is None:

            return

        if not self.vehicle_state.position_valid:

            return

        # Avoid using stale depth.
        if (
            time.monotonic()
            - self.latest_depth_time
            > 0.5
        ):

            return

        depth = self.latest_depth

        height, width = depth.shape[:2]

        u = int(
            round(
                detection.center_u_norm
                * (width - 1)
            )
        )

        v = int(
            round(
                detection.center_v_norm
                * (height - 1)
            )
        )

        u = max(
            0,
            min(width - 1, u),
        )

        v = max(
            0,
            min(height - 1, v),
        )

        z_depth = self._median_depth(
            depth,
            u,
            v,
        )

        if z_depth is None:

            return

        # --------------------------------------------------------
        # Depth camera intrinsics
        # --------------------------------------------------------

        fx = float(
            self.depth_info.k[0]
        )

        fy = float(
            self.depth_info.k[4]
        )

        cx = float(
            self.depth_info.k[2]
        )

        cy = float(
            self.depth_info.k[5]
        )

        if (
            fx <= 1e-6
            or fy <= 1e-6
        ):

            self.get_logger().warning(
                'Invalid depth CameraInfo intrinsics.'
            )

            return

        # --------------------------------------------------------
        # ROS optical frame:
        #
        # X_opt = right
        # Y_opt = down
        # Z_opt = forward
        # --------------------------------------------------------

        x_opt = (
            (u - cx)
            * z_depth
            / fx
        )

        y_opt = (
            (v - cy)
            * z_depth
            / fy
        )

        z_opt = z_depth

        # --------------------------------------------------------
        # Optical -> body FLU
        #
        # Body:
        # x = forward
        # y = left
        # z = up
        # --------------------------------------------------------

        x_body = (
            z_opt
            + self.mount_x
        )

        y_body = (
            -x_opt
            + self.mount_y
        )

        z_body = (
            -y_opt
            + self.mount_z
        )

        # --------------------------------------------------------
        # Body horizontal plane -> world ENU.
        #
        # First closed-loop version assumes the vehicle is close
        # to level flight. Roll/pitch compensation will later be
        # upgraded using full VehicleAttitude quaternion.
        # --------------------------------------------------------

        yaw = float(
            self.vehicle_state.yaw
        )

        cos_yaw = math.cos(
            yaw
        )

        sin_yaw = math.sin(
            yaw
        )

        dx_east = (
            cos_yaw * x_body
            - sin_yaw * y_body
        )

        dy_north = (
            sin_yaw * x_body
            + cos_yaw * y_body
        )

        dz_up = z_body

        world_x = (
            float(self.vehicle_state.x)
            + dx_east
        )

        world_y = (
            float(self.vehicle_state.y)
            + dy_north
        )

        world_z = (
            float(self.vehicle_state.z)
            + dz_up
        )

        self.history.append(
            (
                world_x,
                world_y,
                world_z,
                float(detection.confidence),
            )
        )

        self._publish_estimate(
            detection.label
        )

    # ============================================================
    # Depth processing
    # ============================================================

    def _median_depth(
        self,
        depth,
        u,
        v,
    ):

        radius = self.patch_radius

        x0 = max(
            0,
            u - radius,
        )

        x1 = min(
            depth.shape[1],
            u + radius + 1,
        )

        y0 = max(
            0,
            v - radius,
        )

        y1 = min(
            depth.shape[0],
            v + radius + 1,
        )

        patch = depth[
            y0:y1,
            x0:x1
        ].reshape(-1)

        valid = patch[
            np.isfinite(patch)
        ]

        valid = valid[
            (
                valid >= self.min_depth
            )
            &
            (
                valid <= self.max_depth
            )
        ]

        if valid.size == 0:

            return None

        return float(
            np.median(valid)
        )

    # ============================================================
    # Stable world estimate
    # ============================================================

    def _publish_estimate(
        self,
        label,
    ):

        array = np.array(
            [
                [
                    item[0],
                    item[1],
                    item[2],
                ]
                for item in self.history
            ],
            dtype=np.float64,
        )

        confidence_values = [
            item[3]
            for item in self.history
        ]

        median = np.median(
            array,
            axis=0,
        )

        dx = (
            array[:, 0]
            - median[0]
        )

        dy = (
            array[:, 1]
            - median[1]
        )

        radial_error = np.sqrt(
            dx * dx
            + dy * dy
        )

        scatter_xy = float(
            np.median(radial_error)
        )

        stable = bool(
            len(self.history)
            >= self.stable_min_samples
            and scatter_xy
            <= self.stable_scatter_xy
        )

        target = SemanticTarget()

        target.timestamp_us = (
            time.time_ns() // 1000
        )

        target.valid = True
        target.stable = stable

        target.label = str(label)

        target.confidence = float(
            np.mean(confidence_values)
        )

        target.x = float(
            median[0]
        )

        target.y = float(
            median[1]
        )

        target.z = float(
            median[2]
        )

        current_x = float(
            self.vehicle_state.x
        )

        current_y = float(
            self.vehicle_state.y
        )

        current_z = float(
            self.vehicle_state.z
        )

        target.range_m = float(
            math.sqrt(
                (
                    target.x
                    - current_x
                ) ** 2
                +
                (
                    target.y
                    - current_y
                ) ** 2
                +
                (
                    target.z
                    - current_z
                ) ** 2
            )
        )

        target.sample_count = int(
            len(self.history)
        )

        target.scatter_xy = float(
            scatter_xy
        )

        self.target_pub.publish(
            target
        )


def main(args=None):

    rclpy.init(args=args)

    node = TargetMapper()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
