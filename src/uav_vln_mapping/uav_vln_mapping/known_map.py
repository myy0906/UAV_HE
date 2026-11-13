#!/usr/bin/env python3

import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from nav_msgs.msg import OccupancyGrid

from ament_index_python.packages import (
    get_package_share_directory,
)


class KnownMap(Node):

    def __init__(self):

        super().__init__("known_map")

        default_config = str(
            Path(
                get_package_share_directory(
                    "uav_vln_bringup"
                )
            )
            / "config"
            / "known_map.json"
        )

        self.declare_parameter(
            "config_file",
            default_config,
        )

        self.declare_parameter(
            "publish_rate",
            2.0,
        )

        config_file = (
            self.get_parameter(
                "config_file"
            )
            .get_parameter_value()
            .string_value
        )

        publish_rate = float(
            self.get_parameter(
                "publish_rate"
            ).value
        )

        if (
            not math.isfinite(publish_rate)
            or publish_rate <= 0
        ):
            raise ValueError(
                "Invalid publish_rate"
            )

        with open(
            config_file,
            "r",
            encoding="utf-8",
        ) as file:

            self.config = json.load(file)

        self.frame_id = self.config[
            "frame_id"
        ]

        self.resolution = float(
            self.config["resolution"]
        )

        bounds = self.config["bounds"]

        self.xmin = float(bounds["xmin"])
        self.xmax = float(bounds["xmax"])
        self.ymin = float(bounds["ymin"])
        self.ymax = float(bounds["ymax"])

        values = [
            self.resolution,
            self.xmin,
            self.xmax,
            self.ymin,
            self.ymax,
        ]

        if not all(
            math.isfinite(value)
            for value in values
        ):
            raise ValueError(
                "Map contains non-finite values"
            )

        if self.resolution <= 0:
            raise ValueError(
                "Map resolution must be positive"
            )

        if (
            self.xmax <= self.xmin
            or self.ymax <= self.ymin
        ):
            raise ValueError(
                "Invalid map bounds"
            )

        self.width = int(
            math.ceil(
                (self.xmax - self.xmin)
                / self.resolution
            )
        )

        self.height = int(
            math.ceil(
                (self.ymax - self.ymin)
                / self.resolution
            )
        )

        if (
            self.width * self.height
            > 10000000
        ):
            raise ValueError(
                "Map is too large"
            )

        self.obstacles = self.config[
            "obstacles"
        ]

        # All cells start as known free space.
        self.grid_data = [
            0
        ] * (
            self.width * self.height
        )

        self.build_grid()

        # Keep the latest static map available
        # for subscribers that start later.
        qos = QoSProfile(
            depth=1,
            reliability=(
                ReliabilityPolicy.RELIABLE
            ),
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        self.publisher = self.create_publisher(
            OccupancyGrid,
            "/uav/navigation/occupancy_grid",
            qos,
        )

        self.map_msg = (
            self.create_map_message()
        )

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_map,
        )

        occupied_cells = sum(
            1
            for value in self.grid_data
            if value == 100
        )

        self.get_logger().info(
            "Known static map initialized."
        )

        self.get_logger().info(
            f"Config: {config_file}"
        )

        self.get_logger().info(
            f"Map size: "
            f"{self.width} x "
            f"{self.height} cells"
        )

        self.get_logger().info(
            f"Resolution: "
            f"{self.resolution:.2f} m"
        )

        self.get_logger().info(
            f"Known obstacles: "
            f"{len(self.obstacles)}"
        )

        self.get_logger().info(
            f"Occupied cells: "
            f"{occupied_cells}"
        )

        self.get_logger().info(
            "LiDAR is not required."
        )

    def build_grid(self):

        for obstacle in self.obstacles:

            oxmin = float(
                obstacle["xmin"]
            )

            oxmax = float(
                obstacle["xmax"]
            )

            oymin = float(
                obstacle["ymin"]
            )

            oymax = float(
                obstacle["ymax"]
            )

            values = [
                oxmin,
                oxmax,
                oymin,
                oymax,
            ]

            if not all(
                math.isfinite(value)
                for value in values
            ):
                raise ValueError(
                    "Invalid obstacle coordinates"
                )

            if (
                oxmax <= oxmin
                or oymax <= oymin
            ):
                raise ValueError(
                    "Invalid obstacle bounds: "
                    + obstacle["name"]
                )

            # Conservative rasterization:
            # mark every cell intersecting
            # the rectangular obstacle.
            ix_min = max(
                0,
                math.floor(
                    (oxmin - self.xmin)
                    / self.resolution
                ),
            )

            ix_max = min(
                self.width - 1,
                math.ceil(
                    (oxmax - self.xmin)
                    / self.resolution
                ) - 1,
            )

            iy_min = max(
                0,
                math.floor(
                    (oymin - self.ymin)
                    / self.resolution
                ),
            )

            iy_max = min(
                self.height - 1,
                math.ceil(
                    (oymax - self.ymin)
                    / self.resolution
                ) - 1,
            )

            for iy in range(
                iy_min,
                iy_max + 1,
            ):

                for ix in range(
                    ix_min,
                    ix_max + 1,
                ):

                    index = (
                        iy * self.width + ix
                    )

                    self.grid_data[index] = 100

            self.get_logger().info(
                "Loaded obstacle: "
                f"{obstacle['name']}"
            )

    def create_map_message(self):

        msg = OccupancyGrid()

        msg.header.frame_id = (
            self.frame_id
        )

        msg.info.resolution = (
            self.resolution
        )

        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = (
            self.xmin
        )

        msg.info.origin.position.y = (
            self.ymin
        )

        msg.info.origin.position.z = 0.0

        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.info.map_load_time = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.data = self.grid_data

        return msg

    def publish_map(self):

        self.map_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        self.publisher.publish(
            self.map_msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = KnownMap()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
