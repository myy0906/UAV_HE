#!/usr/bin/env python3

import csv
import json
import math
import threading
import time

from datetime import datetime
from pathlib import Path

import rclpy

from rclpy.node import Node

from rclpy.action import (
    ActionClient,
    ActionServer,
    GoalResponse,
    CancelResponse,
)

from rclpy.callback_groups import (
    ReentrantCallbackGroup,
)

from rclpy.executors import (
    MultiThreadedExecutor,
)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage

from ament_index_python.packages import (
    get_package_share_directory,
)

from uav_vln_interfaces.msg import (
    VehicleState,
    SemanticTarget,
)

from uav_vln_interfaces.action import (
    ExecuteMission,
    ExecuteFlightAction,
)

from uav_vln_llm.structured_parser import (
    call_llm,
)

from uav_vln_mission.mission_protocol import (
    ProtocolError,
    check_instruction_scope,
    strict_json_loads,
    validate_mission,
    verify_constraints,
    parse_rule,
)

from uav_vln_navigation.waypoint_compiler import (
    SafetyError,
    SafetyChecker,
    load_config,
    compile_mission,
    rectangle_distance,
)


class MissionCanceled(RuntimeError):
    pass


class MissionManager(Node):

    def __init__(self):

        super().__init__(
            "mission_manager"
        )

        config_file = str(

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
            config_file,
        )

        self.declare_parameter(
            "parser_mode",
            "llm",
        )

        self.declare_parameter(
            "base_url",
            "http://192.168.189.1:1234/v1",
        )

        self.declare_parameter(
            "model",
            "qwen/qwen3-vl-8b",
        )

        self.declare_parameter(
            "llm_timeout_s",
            90.0,
        )

        self.declare_parameter(
            "target_timeout_s",
            30.0,
        )

        self.declare_parameter(
            "target_freshness_s",
            1.5,
        )

        self.config = load_config(

            self.get_parameter(
                "config_file"
            ).value
        )

        self.group = (
            ReentrantCallbackGroup()
        )

        self.lock = threading.RLock()

        self.state = None
        self.state_rx = 0.0

        self.target = None
        self.target_rx = 0.0

        self.busy = False

        self.active_flight = None

        # If flight-action state is uncertain,
        # do not start a second flight action.
        self.flight_uncertain = False

        self.active_goal = None

        self.stage = "IDLE"

        self.run_dir = None
        self.run_start = 0.0

        self.samples = []

        self.report = {}

        self.checker = SafetyChecker(
            self.config
        )

        self.state_sub = (
            self.create_subscription(

                VehicleState,

                "/uav/vehicle_state",

                self.state_callback,

                10,

                callback_group=self.group,
            )
        )

        self.target_sub = (
            self.create_subscription(

                SemanticTarget,

                "/uav/mapping/semantic_target",

                self.target_callback,

                10,

                callback_group=self.group,
            )
        )

        self.path_pub = (
            self.create_publisher(

                PathMessage,

                "/uav/navigation/planned_path",

                10,
            )
        )

        self.flight = ActionClient(

            self,

            ExecuteFlightAction,

            "/uav/execute_flight_action",

            callback_group=self.group,
        )

        self.server = ActionServer(

            self,

            ExecuteMission,

            "/mission/execute",

            execute_callback=self.execute,

            goal_callback=self.goal_callback,

            cancel_callback=self.cancel_callback,

            callback_group=self.group,
        )

        self.get_logger().info(
            "Known-map mission manager ready."
        )

        self.get_logger().info(
            "Static geometry + RGB-D target "
            "+ strict JSON + safe A*."
        )

        self.get_logger().info(
            "No LiDAR subscription."
        )

    def state_callback(self, msg):

        now = time.monotonic()

        with self.lock:

            self.state = msg

            self.state_rx = now

            if self.run_dir is not None:

                values = (

                    msg.x,
                    msg.y,
                    msg.z,

                    msg.vx,
                    msg.vy,
                    msg.vz,
                )

                if all(
                    math.isfinite(float(v))
                    for v in values
                ):

                    self.samples.append(

                        [

                            now - self.run_start,

                            self.stage,

                            float(msg.x),

                            float(msg.y),

                            float(msg.z),

                            float(msg.vx),

                            float(msg.vy),

                            float(msg.vz),

                            int(msg.armed),

                            int(msg.connected),

                            int(msg.position_valid),

                            int(msg.failsafe),

                            int(msg.nav_state),
                        ]
                    )

    def target_callback(self, msg):

        with self.lock:

            self.target = msg

            self.target_rx = (
                time.monotonic()
            )

    def goal_callback(self, request):

        with self.lock:

            if (
                self.busy

                or self.flight_uncertain
            ):

                return GoalResponse.REJECT

            self.busy = True

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal):

        return CancelResponse.ACCEPT

    def check_cancel(self):

        goal = self.active_goal

        if (
            goal is not None

            and goal.is_cancel_requested
        ):

            raise MissionCanceled(
                "Mission canceled by operator"
            )

    def feedback(
        self,
        stage,
        detail,
        progress,
    ):

        with self.lock:

            self.stage = stage

        self.get_logger().info(
            f"[{stage}] {detail}"
        )

        if self.run_dir is not None:

            path = (
                self.run_dir
                / "events.log"
            )

            with path.open(

                "a",

                encoding="utf-8",

            ) as file:

                elapsed = (

                    time.monotonic()

                    - self.run_start
                )

                file.write(

                    f"{elapsed:.3f} "

                    f"[{stage}] "

                    f"{detail}\n"
                )

        goal = self.active_goal

        if goal is not None:

            msg = ExecuteMission.Feedback()

            msg.stage = stage

            msg.detail = str(detail)

            msg.progress = float(
                progress
            )

            goal.publish_feedback(msg)

    def save_json(self, name, data):

        if self.run_dir is None:
            return

        path = (
            self.run_dir / name
        )

        path.write_text(

            json.dumps(

                data,

                ensure_ascii=False,

                indent=2,

                allow_nan=False,
            ),

            encoding="utf-8",
        )

    def position(self):

        with self.lock:

            msg = self.state

            age = (

                time.monotonic()

                - self.state_rx
            )

        if msg is None or age > 1.5:

            raise RuntimeError(
                "Vehicle state missing or stale"
            )

        if not (
            msg.connected
            and msg.position_valid
        ):

            raise RuntimeError(
                "PX4 position or connection invalid"
            )

        p = (

            float(msg.x),

            float(msg.y),

            float(msg.z),
        )

        if not all(
            math.isfinite(v)
            for v in p
        ):

            raise RuntimeError(
                "Invalid vehicle position"
            )

        return p

    def wait_system_ready(self):

        deadline = (

            time.monotonic()

            + 20.0
        )

        while (
            time.monotonic()
            < deadline
        ):

            self.check_cancel()

            try:

                self.position()

                with self.lock:

                    state = self.state

                if (

                    state.pre_flight_checks_pass

                    and not state.failsafe

                    and not state.armed

                    and self.flight.server_is_ready()
                ):

                    return

            except RuntimeError:

                pass

            time.sleep(0.1)

        raise RuntimeError(
            "System readiness timeout"
        )

    def parse_instruction(self, instruction):

        mode = str(

            self.get_parameter(
                "parser_mode"
            ).value
        )

        self.report["parser_mode"] = mode

        if mode not in {
            "llm",
            "rule",
        }:

            raise ProtocolError(
                "Invalid parser_mode"
            )

        start = time.monotonic()

        if mode == "llm":

            # Save the original model response
            # before attempting JSON validation.
            info = call_llm(

                instruction,

                self.get_parameter(
                    "base_url"
                ).value,

                self.get_parameter(
                    "model"
                ).value,

                self.get_parameter(
                    "llm_timeout_s"
                ).value,
            )

            self.save_json(
                "model_response.json",
                info,
            )

            (
                self.run_dir
                / "model_raw_output.txt"
            ).write_text(

                info["raw_output"],

                encoding="utf-8",
            )

            self.report[
                "inference_time_s"
            ] = info[
                "inference_time_s"
            ]

            plan = strict_json_loads(
                info["raw_output"]
            )

            self.save_json(
                "candidate_plan.json",
                plan,
            )

            if (
                isinstance(plan, dict)

                and plan.get("accepted")
                    is False
            ):

                raise ProtocolError(

                    "Model rejected mission: "

                    + str(
                        plan.get("reason")
                    )
                )

        else:

            plan = parse_rule(
                instruction
            )

            self.save_json(
                "candidate_plan.json",
                plan,
            )

            self.report[
                "inference_time_s"
            ] = 0.0

        check_instruction_scope(
            instruction
        )

        validate_mission(
            plan,
            self.config,
        )

        verify_constraints(

            instruction,

            plan,
        )

        self.report[
            "parse_validation_time_s"
        ] = (
            time.monotonic()
            - start
        )

        self.save_json(
            "validated_semantic_plan.json",
            plan,
        )

        self.report[
            "semantic_validation_passed"
        ] = True

        return plan

    def wait_target(self):

        start = time.monotonic()

        timeout = float(

            self.get_parameter(
                "target_timeout_s"
            ).value
        )

        freshness = float(

            self.get_parameter(
                "target_freshness_s"
            ).value
        )

        # Let the existing RGB-D median
        # window refresh after takeoff.
        settle_until = start + 3.0

        while (
            time.monotonic() - start
            < timeout
        ):

            self.check_cancel()

            self.monitor_state(
                "LOCALIZE"
            )

            with self.lock:

                target = self.target

                received = self.target_rx

            now = time.monotonic()

            if (

                now >= settle_until

                and target is not None

                and received >= start

                and now - received
                    <= freshness

                and target.valid

                and target.stable

                and target.label
                    == "red_house"

                and target.sample_count >= 5

                and target.scatter_xy <= 0.75

                and all(

                    math.isfinite(float(v))

                    for v in (

                        target.x,

                        target.y,

                        target.z,

                        target.confidence,
                    )
                )
            ):

                position = (

                    float(target.x),

                    float(target.y),

                    float(target.z),
                )

                self.save_json(

                    "visual_target.json",

                    {

                        "source": "RGB-D",

                        "label":
                            target.label,

                        "position":
                            list(position),

                        "confidence":
                            float(
                                target.confidence
                            ),

                        "sample_count":
                            int(
                                target.sample_count
                            ),

                        "scatter_xy":
                            float(
                                target.scatter_xy
                            ),

                        "observation_position":
                            list(
                                self.position()
                            ),
                    },
                )

                return position

            time.sleep(0.1)

        raise RuntimeError(
            "Visual target localization timeout"
        )

    def monitor_state(self, phase):

        position = self.position()

        with self.lock:

            state = self.state

        if state.failsafe:

            raise SafetyError(
                "PX4 failsafe detected"
            )

        allow_ground = phase in {

            "TAKEOFF",

            "LAND",

            "RECOVERY_LAND",
        }

        self.checker.check_point(

            position,

            allow_ground=allow_ground,
        )

        speed = math.hypot(

            state.vx,

            state.vy,
        )

        vz = abs(
            state.vz
        )

        if not (
            math.isfinite(speed)
            and math.isfinite(vz)
        ):

            raise SafetyError(
                "Invalid velocity estimate"
            )

        # Runtime monitoring tolerances.
        # Strict nominal-limit exceedances
        # are also evaluated from the log.
        limits = self.config[
            "limits"
        ]

        if phase != "RECOVERY_LAND" and speed > (

            limits[
                "horizontal_speed_max"
            ]

            + 0.3
        ):

            raise SafetyError(
                "Horizontal speed exceeded "
                "runtime safety threshold"
            )

        if phase != "RECOVERY_LAND" and vz > (

            limits[
                "vertical_speed_max"
            ]

            + 0.3
        ):

            raise SafetyError(
                "Vertical speed exceeded "
                "runtime safety threshold"
            )

    def wait_future(
        self,
        future,
        timeout,
        monitor=False,
        ignore_cancel=False,
        phase="GOTO",
    ):

        deadline = (
            time.monotonic()
            + timeout
        )

        while not future.done():

            if not ignore_cancel:

                self.check_cancel()

            if monitor:

                self.monitor_state(
                    phase
                )

            if (
                time.monotonic()
                >= deadline
            ):

                raise RuntimeError(
                    "ROS2 action future timeout"
                )

            time.sleep(0.05)

        return future.result()

    def stop_active_flight(self):

        handle = self.active_flight

        if handle is None:

            return not self.flight_uncertain

        try:

            response = self.wait_future(

                handle.cancel_goal_async(),

                timeout=5.0,

                ignore_cancel=True,
            )

            # Wait for terminal result before
            # allowing any subsequent action.
            wrapped = self.wait_future(

                handle.get_result_async(),

                timeout=15.0,

                ignore_cancel=True,
            )

            self.active_flight = None

            self.flight_uncertain = False

            return True

        except Exception as exc:

            self.flight_uncertain = True

            self.get_logger().error(

                "Flight cancellation not "
                f"confirmed: {exc}"
            )

            return False

    def flight_action(
        self,
        action_type,
        target=None,
        yaw=float("nan"),
        duration=0.0,
        timeout=30.0,
        phase="GOTO",
        ignore_cancel=False,
    ):

        if self.flight_uncertain:

            raise RuntimeError(
                "Previous flight action state "
                "is uncertain"
            )

        if not ignore_cancel:

            self.check_cancel()

        goal = ExecuteFlightAction.Goal()

        goal.action_type = (
            action_type
        )

        if target is not None:

            goal.x = float(
                target[0]
            )

            goal.y = float(
                target[1]
            )

            goal.z = float(
                target[2]
            )

        goal.yaw = float(yaw)

        goal.duration = float(
            duration
        )

        goal.timeout = float(
            timeout
        )

        goal.acceptance_radius = float(

            self.config["limits"][
                "waypoint_acceptance_radius"
            ]
        )

        if action_type == (
            ExecuteFlightAction.Goal.TAKEOFF
        ):

            goal.acceptance_radius = 0.15

        send_future = (
            self.flight.send_goal_async(
                goal
            )
        )

        # Once a goal has been sent, obtain
        # its acceptance response before
        # attempting cancellation or recovery.
        self.flight_uncertain = True

        handle = self.wait_future(

            send_future,

            timeout=5.0,

            ignore_cancel=True,
        )

        self.flight_uncertain = False

        if not handle.accepted:

            raise RuntimeError(
                "PX4 executor rejected action"
            )

        self.active_flight = handle

        try:

            wrapped = self.wait_future(

                handle.get_result_async(),

                timeout=timeout + 10.0,

                monitor=True,

                ignore_cancel=ignore_cancel,

                phase=phase,
            )

            self.active_flight = None

            result = wrapped.result

            if not result.success:

                raise RuntimeError(
                    result.message
                )

            return result

        except Exception:

            self.stop_active_flight()

            raise

    def publish_path(
        self,
        points,
    ):

        msg = PathMessage()

        msg.header.frame_id = "map"

        msg.header.stamp = (

            self.get_clock()

            .now()

            .to_msg()
        )

        for point in points:

            pose = PoseStamped()

            pose.header = msg.header

            pose.pose.position.x = (
                float(point[0])
            )

            pose.pose.position.y = (
                float(point[1])
            )

            pose.pose.position.z = (
                float(point[2])
            )

            pose.pose.orientation.w = (
                1.0
            )

            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def execute_path(
        self,
        points,
        yaw,
        total_timeout,
        phase,
        progress_start,
        progress_end,
    ):

        self.publish_path(
            points
        )

        deadline = (

            time.monotonic()

            + total_timeout
        )

        for index, waypoint in enumerate(
            points
        ):

            self.check_cancel()

            current = self.position()

            # Verify every actual execution
            # segment, not just the original
            # ideal planned segment.
            self.checker.check_segment(

                current,

                waypoint,
            )

            distance = math.dist(

                current,

                waypoint,
            )

            if distance < 0.10:

                continue

            remaining = (

                deadline

                - time.monotonic()
            )

            speed = self.config[
                "limits"
            ][
                "horizontal_speed_max"
            ]

            if remaining < (

                distance / speed

                + 3.0
            ):

                raise SafetyError(
                    f"{phase}: "
                    "remaining time infeasible"
                )

            # Gazebo may run slower than wall time.
            # Give each waypoint sufficient time,
            # while preserving the mission deadline.

            action_timeout = min(

                remaining,

                max(

                    30.0,

                    distance / (0.20 * speed)

                    + 12.0,
                ),
            )

            progress = (

                progress_start

                + (

                    progress_end

                    - progress_start
                )

                * index

                / max(1, len(points))
            )

            self.feedback(

                phase,

                f"Waypoint "
                f"{index + 1}/{len(points)} "
                f"{waypoint}",

                progress,
            )

            self.flight_action(

                ExecuteFlightAction.Goal.GOTO,

                target=waypoint,

                yaw=yaw,

                timeout=action_timeout,

                phase=phase,
            )

        error = math.dist(

            self.position(),

            points[-1],
        )

        tolerance = (

            self.config["limits"][
                "waypoint_acceptance_radius"
            ]

            + 0.10
        )

        if error > tolerance:

            raise RuntimeError(

                f"{phase}: "
                f"endpoint error {error:.3f} m"
            )

    def recover(self):

        if not self.stop_active_flight():

            return (

                "Flight cancellation uncertain. "
                "No new action sent. "
                "Operator intervention required."
            )

        with self.lock:

            state = self.state

        if state is None:

            return (
                "Vehicle state unavailable"
            )

        if not state.armed:

            return (
                "Vehicle already disarmed"
            )

        try:

            current = self.position()

            # Never perform an unchecked
            # direct RETURN_HOME action.
            # Land at the current location
            # only if the vertical column
            # satisfies known-map constraints.
            self.checker.check_segment(

                current,

                (
                    current[0],

                    current[1],

                    0.0,
                ),

                allow_ground=True,
            )

            self.feedback(

                "RECOVERY_LAND",

                "Landing in validated "
                "current-position column",

                0.95,
            )

            self.flight_action(

                ExecuteFlightAction.Goal.LAND,

                timeout=45.0,

                phase="RECOVERY_LAND",

                ignore_cancel=True,
            )

            return (
                "Recovery landing completed"
            )

        except Exception as exc:

            return (

                "Automatic recovery not completed: "

                + str(exc)

                + ". Check PX4 state; "

                "operator intervention required."
            )

    def save_trajectory(self):

        with self.lock:

            rows = list(
                self.samples
            )

        path = (
            self.run_dir
            / "trajectory.csv"
        )

        with path.open(

            "w",

            newline="",

            encoding="utf-8",

        ) as file:

            writer = csv.writer(
                file
            )

            writer.writerow(

                [

                    "t_s",

                    "stage",

                    "x",

                    "y",

                    "z",

                    "vx",

                    "vy",

                    "vz",

                    "armed",

                    "connected",

                    "position_valid",

                    "failsafe",

                    "nav_state",
                ]
            )

            writer.writerows(rows)

        return rows

    def compute_metrics(self, rows):

        valid = [

            row

            for row in rows

            if (

                row[8]

                and row[9]

                and row[10]
            )
        ]

        metrics = {

            "success":
                self.report.get(
                    "success",
                    False,
                ),

            "trajectory_samples":
                len(rows),

            "armed_valid_samples":
                len(valid),

            "inference_time_s":
                self.report.get(
                    "inference_time_s"
                ),

            "compile_time_s":
                self.report.get(
                    "compile_time_s"
                ),

            "navigation_error_m":
                self.report.get(
                    "navigation_error_m"
                ),

            "home_error_m":
                self.report.get(
                    "home_error_m"
                ),

            "mission_wall_time_s":
                time.monotonic()
                - self.run_start,
        }

        if not valid:

            return metrics

        points = [

            tuple(row[2:5])

            for row in valid
        ]

        metrics[
            "flight_time_s"
        ] = (

            valid[-1][0]

            - valid[0][0]
        )

        metrics[
            "actual_path_length_m"
        ] = sum(

            math.dist(a, b)

            for a, b in zip(
                points,
                points[1:],
            )
        )

        metrics[
            "max_horizontal_speed_mps"
        ] = max(

            math.hypot(
                row[5],
                row[6],
            )

            for row in valid
        )

        metrics[
            "max_vertical_speed_mps"
        ] = max(

            abs(row[7])

            for row in valid
        )

        obstacles = self.config[
            "obstacles"
        ]

        distances = [

            rectangle_distance(

                point[0],

                point[1],

                obstacle,
            )

            for point in points

            for obstacle in obstacles
        ]

        metrics[
            "min_known_obstacle_distance_m"
        ] = (

            min(distances)

            if distances

            else None
        )

        fence = self.config[
            "geofence"
        ]

        outside = [

            row

            for row in valid

            if not (

                fence["xmin"]
                <= row[2]
                <= fence["xmax"]

                and fence["ymin"]
                <= row[3]
                <= fence["ymax"]

                and -0.3
                <= row[4]
                <= fence["zmax"]
            )
        ]

        metrics[
            "geofence_violation"
        ] = bool(outside)

        cruise_rows = [

            row

            for row in valid

            if row[1] in {

                "LOCALIZE",

                "COMPILE",

                "OUTBOUND",

                "HOVER",

                "RETURN",
            }
        ]

        metrics[
            "cruise_altitude_violation"
        ] = any(

            row[4] < fence["zmin"]

            for row in cruise_rows
        )

        nominal_speed = self.config[
            "limits"
        ][
            "horizontal_speed_max"
        ]

        metrics[
            "nominal_speed_violation"
        ] = (

            metrics[
                "max_horizontal_speed_mps"
            ]

            > nominal_speed
        )

        metrics[
            "known_obstacle_margin_violation"
        ] = bool(

            distances

            and min(distances)

            < self.config[
                "limits"
            ][
                "clearance"
            ]
        )

        # This is a distance-based proxy,
        # not an actual Gazebo contact event.
        metrics[
            "known_obstacle_center_penetration"
        ] = bool(

            distances

            and min(distances) <= 0.0
        )

        return metrics

    def execute(self, goal):

        self.active_goal = goal

        self.checker = SafetyChecker(
            self.config
        )

        instruction = (
            goal.request.instruction
        )

        mission_id = (

            datetime.now()

            .strftime(
                "%Y%m%d_%H%M%S_%f"
            )
        )

        directory = (

            Path.home()

            / "px4_ros2_ws"

            / "results"

            / mission_id
        )

        directory.mkdir(

            parents=True,

            exist_ok=False,
        )

        with self.lock:

            self.samples = []

            self.run_start = (
                time.monotonic()
            )

            self.run_dir = directory

        self.report = {

            "mission_id":
                mission_id,

            "instruction":
                instruction,

            "semantic_validation_passed":
                False,

            "path_validation_passed":
                False,

            "flight_started":
                False,

            "success":
                False,

            "failure_category":
                None,

            "failure_reason":
                None,
        }

        self.save_json(

            "scene_config.json",

            self.config,
        )

        (
            directory
            / "instruction.txt"
        ).write_text(

            instruction,

            encoding="utf-8",
        )

        failure_category = "SYSTEM"

        canceled = False

        try:

            self.feedback(

                "UNDERSTAND",

                "Parsing candidate mission",

                0.02,
            )

            failure_category = "LANGUAGE"

            semantic_plan = (
                self.parse_instruction(
                    instruction
                )
            )

            self.check_cancel()

            actions = {

                action["action_type"]:
                    action

                for action in
                    semantic_plan[
                        "actions"
                    ]
            }

            self.feedback(

                "WAIT_SYSTEM",

                "Waiting for PX4 and executor",

                0.10,
            )

            failure_category = "SYSTEM"

            self.wait_system_ready()

            home = self.position()

            self.report[
                "home"
            ] = list(home)

            if math.hypot(

                home[0],

                home[1],

            ) > 0.5:

                raise SafetyError(
                    "Home not aligned with "
                    "configured map origin"
                )

            altitude = float(

                actions["takeoff"][
                    "altitude"
                ]
            )

            takeoff_point = (

                home[0],

                home[1],

                altitude,
            )

            failure_category = "SAFETY"

            self.checker.check_segment(

                home,

                takeoff_point,

                allow_ground=True,
            )

            self.feedback(

                "TAKEOFF",

                f"Validated takeoff "
                f"to {altitude:.2f} m",

                0.15,
            )

            failure_category = "EXECUTION"

            self.report[
                "flight_started"
            ] = True

            self.flight_action(

                ExecuteFlightAction.Goal.TAKEOFF,

                target=takeoff_point,

                timeout=float(

                    actions["takeoff"][
                        "timeout"
                    ]
                ),

                phase="TAKEOFF",
            )

            self.feedback(

                "LOCALIZE",

                "Waiting for stable RGB-D "
                "red-house localization",

                0.25,
            )

            failure_category = "PERCEPTION"

            target = self.wait_target()

            self.feedback(

                "COMPILE",

                "Compiling complete outbound "
                "and return paths",

                0.32,
            )

            failure_category = "PLANNING"

            safe_plan = compile_mission(

                semantic_plan,

                home,

                self.position(),

                target,

                self.config,
            )

            safe_plan[
                "mission_id"
            ] = mission_id

            safe_plan[
                "semantic_plan"
            ] = semantic_plan

            safe_plan[
                "fallback_action"
            ] = (
                "cancel_and_land_if_safe"
            )

            self.checker = SafetyChecker(

                self.config,

                safe_plan[
                    "target_guard"
                ],
            )

            self.save_json(

                "safe_plan.json",

                safe_plan,
            )

            self.report[
                "path_validation_passed"
            ] = True

            self.report[
                "compile_time_s"
            ] = safe_plan[
                "compile_time_s"
            ]

            self.save_json(

                "validation_report.json",

                {

                    "accepted": True,

                    "semantic": True,

                    "geofence": True,

                    "obstacle_segments": True,

                    "target_guard": True,

                    "reachability": True,

                    "outbound_waypoints":

                        len(

                            safe_plan[
                                "outbound_waypoints"
                            ]
                        ),

                    "return_waypoints":

                        len(

                            safe_plan[
                                "return_waypoints"
                            ]
                        ),
                },
            )

            failure_category = "EXECUTION"

            self.execute_path(

                safe_plan[
                    "outbound_waypoints"
                ],

                yaw=safe_plan["yaw"],

                total_timeout=float(

                    actions["goto"][
                        "timeout"
                    ]
                ),

                phase="OUTBOUND",

                progress_start=0.35,

                progress_end=0.60,
            )

            hover_position = (
                self.position()
            )

            if "hover" in actions:

                duration = float(

                    actions["hover"][
                        "duration"
                    ]
                )

                self.feedback(

                    "HOVER",

                    f"Observing for "
                    f"{duration:.1f} seconds",

                    0.65,
                )

                self.flight_action(

                    ExecuteFlightAction.Goal.HOVER,

                    duration=duration,

                    timeout=duration + 15.0,

                    phase="HOVER",
                )

                hover_position = (
                    self.position()
                )

            self.report[
                "hover_position"
            ] = list(
                hover_position
            )

            self.report[
                "navigation_error_m"
            ] = math.dist(

                hover_position,

                safe_plan[
                    "approach_point"
                ],
            )

            self.execute_path(

                safe_plan[
                    "return_waypoints"
                ],

                yaw=float("nan"),

                total_timeout=float(

                    actions["return_home"][
                        "timeout"
                    ]
                ),

                phase="RETURN",

                progress_start=0.70,

                progress_end=0.90,
            )

            current = self.position()

            self.checker.check_segment(

                current,

                (

                    current[0],

                    current[1],

                    0.0,
                ),

                allow_ground=True,
            )

            self.feedback(

                "LAND",

                "Landing at home",

                0.94,
            )

            self.flight_action(

                ExecuteFlightAction.Goal.LAND,

                timeout=float(

                    actions["land"][
                        "timeout"
                    ]
                ),

                phase="LAND",
            )

            final = self.position()

            self.report[
                "home_error_m"
            ] = math.hypot(

                final[0] - home[0],

                final[1] - home[1],
            )

            self.report[
                "success"
            ] = True

            self.feedback(

                "COMPLETED",

                "Mission completed",

                1.0,
            )

        except MissionCanceled as exc:

            canceled = True

            self.report[
                "failure_category"
            ] = "CANCELED"

            self.report[
                "failure_reason"
            ] = str(exc)

        except Exception as exc:

            if isinstance(
                exc,
                SafetyError,
            ):

                failure_category = (
                    "SAFETY"

                    if failure_category
                    != "PLANNING"

                    else "PLANNING"
                )

            self.report[
                "failure_category"
            ] = failure_category

            self.report[
                "failure_reason"
            ] = str(exc)

        if not self.report[
            "success"
        ]:

            self.feedback(

                "FAILED",

                str(

                    self.report[
                        "failure_reason"
                    ]
                ),

                0.0,
            )

            if not self.report[
                "path_validation_passed"
            ]:

                self.save_json(

                    "validation_report.json",

                    {

                        "accepted": False,

                        "reason":

                            self.report[
                                "failure_reason"
                            ],

                        "category":

                            self.report[
                                "failure_category"
                            ],
                    },
                )

            if self.report[
                "flight_started"
            ]:

                self.report[
                    "recovery"
                ] = self.recover()

                self.feedback(

                    "RECOVERY_RESULT",

                    self.report[
                        "recovery"
                    ],

                    0.98,
                )

        result = ExecuteMission.Result()

        result.success = bool(

            self.report[
                "success"
            ]
        )

        result.message = (

            "Mission completed"

            if result.success

            else str(

                self.report[
                    "failure_reason"
                ]
            )
        )

        try:

            rows = self.save_trajectory()

            metrics = (
                self.compute_metrics(rows)
            )

            self.save_json(

                "metrics.json",

                metrics,
            )

            self.save_json(

                "mission_report.json",

                self.report,
            )

        except Exception as exc:

            result.success = False

            result.message += (

                f"; logging error: {exc}"
            )

        result.parsed_json = json.dumps(

            {

                "mission_id":
                    mission_id,

                "result_directory":
                    str(directory),

                "report":
                    self.report,
            },

            ensure_ascii=False,
        )

        if canceled:

            goal.canceled()

        elif result.success:

            goal.succeed()

        else:

            goal.abort()

        with self.lock:

            self.run_dir = None

            self.active_goal = None

            self.busy = False

            self.stage = "IDLE"

        return result


def main(args=None):

    rclpy.init(
        args=args
    )

    node = MissionManager()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:

        executor.spin()

    except KeyboardInterrupt:

        pass

    finally:

        executor.shutdown()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
