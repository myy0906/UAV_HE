import math
import threading
import time

import rclpy

from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleStatus,
)

from uav_vln_interfaces.action import ExecuteFlightAction
from uav_vln_interfaces.msg import VehicleState


# =============================================================================
# Coordinate conversion
# =============================================================================


def wrap_pi(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def enu_to_ned_position(x_e, y_n, z_u):
    """
    ROS ENU -> PX4 NED.

    ENU:
        x = East
        y = North
        z = Up

    NED:
        x = North
        y = East
        z = Down
    """
    return (
        y_n,
        x_e,
        -z_u,
    )


def ned_to_enu_position(x_n, y_e, z_d):
    """PX4 NED -> ROS ENU."""
    return (
        y_e,
        x_n,
        -z_d,
    )


def yaw_enu_to_ned(yaw_enu: float) -> float:
    """ROS ENU yaw -> PX4 NED yaw."""
    return wrap_pi(
        math.pi / 2.0 - yaw_enu
    )


def yaw_ned_to_enu(yaw_ned: float) -> float:
    """PX4 NED yaw -> ROS ENU yaw."""
    return wrap_pi(
        math.pi / 2.0 - yaw_ned
    )


# =============================================================================
# PX4 Executor
# =============================================================================


class Px4Executor(Node):

    CONTROL_PERIOD_S = 0.05
    STATE_PERIOD_S = 0.10

    OFFBOARD_WARMUP_S = 1.2

    DEFAULT_TIMEOUT_S = 30.0
    DEFAULT_ACCEPTANCE_RADIUS_M = 0.40
    DEFAULT_TAKEOFF_ALTITUDE_M = 2.0
    DEFAULT_HOVER_DURATION_S = 3.0

    PX4_CONNECTION_TIMEOUT_S = 1.0

    def __init__(self):

        super().__init__('px4_executor')

        self._callback_group = ReentrantCallbackGroup()

        # ---------------------------------------------------------------------
        # PX4-compatible QoS
        # ---------------------------------------------------------------------

        self._px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------------------------------------------------------------
        # ROS 2 -> PX4
        # ---------------------------------------------------------------------

        self._offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            self._px4_qos,
        )

        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self._px4_qos,
        )

        self._vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            self._px4_qos,
        )

        # ---------------------------------------------------------------------
        # PX4 -> ROS 2
        #
        # PX4 v1.17 uses versioned topics for VehicleLocalPosition
        # and VehicleStatus.
        # ---------------------------------------------------------------------

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._on_local_position,
            self._px4_qos,
            callback_group=self._callback_group,
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self._on_vehicle_status,
            self._px4_qos,
            callback_group=self._callback_group,
        )

        self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self._on_vehicle_command_ack,
            self._px4_qos,
            callback_group=self._callback_group,
        )

        # ---------------------------------------------------------------------
        # Project-level state
        # ---------------------------------------------------------------------

        self._vehicle_state_pub = self.create_publisher(
            VehicleState,
            '/uav/vehicle_state',
            10,
        )

        # ---------------------------------------------------------------------
        # Project-level Action interface
        # ---------------------------------------------------------------------

        self._action_server = ActionServer(
            self,
            ExecuteFlightAction,
            '/uav/execute_flight_action',
            execute_callback=self._execute_action,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        # ---------------------------------------------------------------------
        # Shared state
        # ---------------------------------------------------------------------

        self._state_lock = threading.Lock()
        self._busy_lock = threading.Lock()

        self._position_valid = False

        self._current_enu = [
            0.0,
            0.0,
            0.0,
        ]

        self._current_velocity_enu = [
            0.0,
            0.0,
            0.0,
        ]

        self._current_yaw_enu = 0.0

        self._home_enu = None

        self._vehicle_status = None

        self._last_px4_rx_monotonic = 0.0

        # Latest ACK for each PX4 command.
        #
        # command -> (result, result_param1, result_param2, monotonic_time)
        self._last_ack_by_command = {}

        self._last_control_error = ''

        # ---------------------------------------------------------------------
        # Action state
        # ---------------------------------------------------------------------

        self._busy = False
        self._abort_requested = False

        # ---------------------------------------------------------------------
        # Active Offboard target
        # ---------------------------------------------------------------------

        self._stream_offboard = False

        self._target_enu = None
        self._target_yaw_enu = 0.0

        # ---------------------------------------------------------------------
        # Timers
        #
        # These continue running in separate executor threads while a
        # synchronous Action execute callback waits with time.sleep().
        # ---------------------------------------------------------------------

        self._control_timer = self.create_timer(
            self.CONTROL_PERIOD_S,
            self._control_tick,
            callback_group=self._callback_group,
        )

        self._state_timer = self.create_timer(
            self.STATE_PERIOD_S,
            self._publish_vehicle_state,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            'PX4 executor started.'
        )

        self.get_logger().info(
            'Project frame = ENU; PX4 boundary frame = NED.'
        )

        self.get_logger().info(
            'Waiting for PX4 local position and vehicle status...'
        )

    # =========================================================================
    # PX4 -> ROS callbacks
    # =========================================================================

    def _on_local_position(
        self,
        msg: VehicleLocalPosition,
    ):

        valid = bool(
            msg.xy_valid
            and msg.z_valid
        )

        x, y, z = ned_to_enu_position(
            msg.x,
            msg.y,
            msg.z,
        )

        vx, vy, vz = ned_to_enu_position(
            msg.vx,
            msg.vy,
            msg.vz,
        )

        with self._state_lock:

            self._position_valid = valid

            self._current_enu = [
                float(x),
                float(y),
                float(z),
            ]

            self._current_velocity_enu = [
                float(vx),
                float(vy),
                float(vz),
            ]

            self._current_yaw_enu = (
                yaw_ned_to_enu(
                    float(msg.heading)
                )
            )

            self._last_px4_rx_monotonic = (
                time.monotonic()
            )

            if (
                valid
                and self._home_enu is None
            ):

                self._home_enu = list(
                    self._current_enu
                )

                self.get_logger().info(
                    'Local home captured (ENU): '
                    f'x={self._home_enu[0]:.3f}, '
                    f'y={self._home_enu[1]:.3f}, '
                    f'z={self._home_enu[2]:.3f}'
                )

    def _on_vehicle_status(
        self,
        msg: VehicleStatus,
    ):

        with self._state_lock:

            self._vehicle_status = msg

            self._last_px4_rx_monotonic = (
                time.monotonic()
            )

    def _on_vehicle_command_ack(
        self,
        msg: VehicleCommandAck,
    ):

        command = int(msg.command)
        result = int(msg.result)

        with self._state_lock:

            self._last_ack_by_command[
                command
            ] = (
                result,
                int(msg.result_param1),
                int(msg.result_param2),
                time.monotonic(),
            )

        if result in (
            VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS,
        ):

            self.get_logger().info(
                'PX4 command ACK: '
                f'command={command}, '
                f'result={result}'
            )

        else:

            self.get_logger().warning(
                'PX4 command rejected/failed: '
                f'command={command}, '
                f'result={result}, '
                f'result_param1={msg.result_param1}, '
                f'result_param2={msg.result_param2}'
            )

    # =========================================================================
    # Project-level VehicleState
    # =========================================================================

    def _publish_vehicle_state(self):

        with self._state_lock:

            position_valid = (
                self._position_valid
            )

            position = list(
                self._current_enu
            )

            velocity = list(
                self._current_velocity_enu
            )

            yaw = float(
                self._current_yaw_enu
            )

            status = self._vehicle_status

            last_rx = (
                self._last_px4_rx_monotonic
            )

        msg = VehicleState()

        msg.timestamp_us = self._now_us()

        msg.connected = bool(
            last_rx > 0.0
            and (
                time.monotonic()
                - last_rx
                < self.PX4_CONNECTION_TIMEOUT_S
            )
        )

        msg.position_valid = bool(
            position_valid
        )

        if status is None:

            msg.armed = False
            msg.nav_state = 255
            msg.failsafe = False
            msg.pre_flight_checks_pass = False

        else:

            msg.armed = bool(
                status.arming_state
                == VehicleStatus.ARMING_STATE_ARMED
            )

            msg.nav_state = int(
                status.nav_state
            )

            msg.failsafe = bool(
                status.failsafe
            )

            msg.pre_flight_checks_pass = bool(
                status.pre_flight_checks_pass
            )

        msg.x = float(position[0])
        msg.y = float(position[1])
        msg.z = float(position[2])

        msg.vx = float(velocity[0])
        msg.vy = float(velocity[1])
        msg.vz = float(velocity[2])

        msg.yaw = float(yaw)

        self._vehicle_state_pub.publish(
            msg
        )

    # =========================================================================
    # Continuous Offboard stream
    # =========================================================================

    def _control_tick(self):

        if not self._stream_offboard:
            return

        with self._state_lock:

            if self._target_enu is None:
                return

            target = list(
                self._target_enu
            )

            target_yaw = float(
                self._target_yaw_enu
            )

        # ---------------------------------------------------------------------
        # Offboard heartbeat
        # ---------------------------------------------------------------------

        mode = OffboardControlMode()

        mode.timestamp = self._now_us()

        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.thrust_and_torque = False
        mode.direct_actuator = False

        self._offboard_pub.publish(
            mode
        )

        # ---------------------------------------------------------------------
        # ENU target -> NED target
        # ---------------------------------------------------------------------

        north, east, down = (
            enu_to_ned_position(
                target[0],
                target[1],
                target[2],
            )
        )

        nan = float('nan')

        setpoint = TrajectorySetpoint()

        setpoint.timestamp = self._now_us()

        setpoint.position = [
            float(north),
            float(east),
            float(down),
        ]

        setpoint.velocity = [
            nan,
            nan,
            nan,
        ]

        setpoint.acceleration = [
            nan,
            nan,
            nan,
        ]

        setpoint.jerk = [
            nan,
            nan,
            nan,
        ]

        setpoint.yaw = float(
            yaw_enu_to_ned(
                target_yaw
            )
        )

        setpoint.yawspeed = nan

        self._trajectory_pub.publish(
            setpoint
        )

    # =========================================================================
    # PX4 VehicleCommand
    # =========================================================================

    def _publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    ):

        msg = VehicleCommand()

        msg.timestamp = self._now_us()

        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)

        msg.command = int(command)

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.confirmation = 0
        msg.from_external = True

        with self._state_lock:

            self._last_ack_by_command.pop(
                int(command),
                None,
            )

        self._vehicle_command_pub.publish(
            msg
        )

    # =========================================================================
    # Enter Offboard + Arm
    # =========================================================================

    def _ensure_offboard_and_armed(
        self,
        timeout_s,
    ):

        self._last_control_error = ''

        if not self._has_valid_position():

            self._last_control_error = (
                'PX4 local position is invalid.'
            )

            return False

        if (
            self._is_offboard()
            and self._is_armed()
        ):

            self._stream_offboard = True
            return True

        # ---------------------------------------------------------------------
        # Warn about pre-flight status, but do not hide the actual PX4 ACK.
        # ---------------------------------------------------------------------

        if not self._preflight_checks_pass():

            self.get_logger().warning(
                'PX4 reports '
                'pre_flight_checks_pass=false. '
                'Arming may be rejected. '
                'The ARM command will still be sent '
                'so that the exact PX4 ACK can be observed.'
            )

        # ---------------------------------------------------------------------
        # Before switching to Offboard, stream current-position setpoint.
        # ---------------------------------------------------------------------

        current = self._get_current_enu()

        self._set_target(
            current,
            self._get_current_yaw_enu(),
        )

        self._stream_offboard = True

        self.get_logger().info(
            'Warming up Offboard stream '
            f'for {self.OFFBOARD_WARMUP_S:.1f} s.'
        )

        time.sleep(
            self.OFFBOARD_WARMUP_S
        )

        # ---------------------------------------------------------------------
        # Request Offboard mode.
        # ---------------------------------------------------------------------

        if not self._is_offboard():

            self.get_logger().info(
                'Requesting PX4 OFFBOARD mode.'
            )

            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,
            )

        # Give PX4 a short time to process the mode command.
        time.sleep(0.20)

        # ---------------------------------------------------------------------
        # Request ARM.
        # ---------------------------------------------------------------------

        if not self._is_armed():

            self.get_logger().info(
                'Requesting vehicle ARM.'
            )

            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )

        wait_s = min(
            max(float(timeout_s), 3.0),
            8.0,
        )

        deadline = (
            time.monotonic()
            + wait_s
        )

        while (
            time.monotonic()
            < deadline
        ):

            if (
                self._is_offboard()
                and self._is_armed()
            ):

                self.get_logger().info(
                    'Vehicle is ARMED and in OFFBOARD.'
                )

                return True

            mode_ack = self._get_recent_ack(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            )

            arm_ack = self._get_recent_ack(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            )

            if self._ack_is_failure(
                mode_ack
            ):

                self._last_control_error = (
                    'PX4 rejected OFFBOARD mode request: '
                    f'{self._format_ack(mode_ack)}'
                )

                return False

            if self._ack_is_failure(
                arm_ack
            ):

                self._last_control_error = (
                    'PX4 rejected ARM request: '
                    f'{self._format_ack(arm_ack)}; '
                    'pre_flight_checks_pass='
                    f'{self._preflight_checks_pass()}'
                )

                return False

            time.sleep(0.10)

        self._last_control_error = (
            'Timed out waiting for '
            'ARM + OFFBOARD. '
            f'armed={self._is_armed()}, '
            f'offboard={self._is_offboard()}, '
            'pre_flight_checks_pass='
            f'{self._preflight_checks_pass()}'
        )

        return False

    # =========================================================================
    # ROS Action server
    # =========================================================================

    def _goal_callback(
        self,
        goal_request,
    ):

        if (
            goal_request.action_type
            > ExecuteFlightAction.Goal.ABORT
        ):

            self.get_logger().warning(
                'Rejecting unknown '
                f'action_type={goal_request.action_type}'
            )

            return GoalResponse.REJECT

        with self._busy_lock:
            busy = self._busy

        if (
            busy
            and goal_request.action_type
            != ExecuteFlightAction.Goal.ABORT
        ):

            self.get_logger().warning(
                'Another flight action is active. '
                'Send ABORT before a new action.'
            )

            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(
        self,
        _goal_handle,
    ):

        self._abort_requested = True

        self._hold_current_position()

        return CancelResponse.ACCEPT

    def _execute_action(
        self,
        goal_handle,
    ):

        goal = goal_handle.request

        # ABORT may run concurrently with an active goal.
        if (
            goal.action_type
            == ExecuteFlightAction.Goal.ABORT
        ):

            return self._execute_abort(
                goal_handle
            )

        with self._busy_lock:

            if self._busy:

                goal_handle.abort()

                return self._make_result(
                    False,
                    'Another flight action is active.',
                )

            self._busy = True

        self._abort_requested = False

        try:

            if not self._wait_for_position(
                5.0
            ):

                goal_handle.abort()

                return self._make_result(
                    False,
                    'No valid PX4 local position.',
                )

            if (
                goal.action_type
                == ExecuteFlightAction.Goal.TAKEOFF
            ):

                return self._execute_takeoff(
                    goal_handle
                )

            if (
                goal.action_type
                == ExecuteFlightAction.Goal.GOTO
            ):

                return self._execute_goto(
                    goal_handle
                )

            if (
                goal.action_type
                == ExecuteFlightAction.Goal.HOVER
            ):

                return self._execute_hover(
                    goal_handle
                )

            if (
                goal.action_type
                == ExecuteFlightAction.Goal.RETURN_HOME
            ):

                return self._execute_return_home(
                    goal_handle
                )

            if (
                goal.action_type
                == ExecuteFlightAction.Goal.LAND
            ):

                return self._execute_land(
                    goal_handle
                )

            goal_handle.abort()

            return self._make_result(
                False,
                'Unsupported action type.',
            )

        except Exception as exc:

            self.get_logger().exception(
                'Unhandled exception while '
                'executing flight action.'
            )

            self._hold_current_position()

            goal_handle.abort()

            return self._make_result(
                False,
                f'Executor exception: '
                f'{type(exc).__name__}: {exc}',
            )

        finally:

            with self._busy_lock:
                self._busy = False

    # =========================================================================
    # TAKEOFF
    # =========================================================================

    def _execute_takeoff(
        self,
        goal_handle,
    ):

        goal = goal_handle.request

        altitude = (
            float(goal.z)
            if goal.z > 0.0
            else self.DEFAULT_TAKEOFF_ALTITUDE_M
        )

        timeout_s = self._goal_timeout(
            goal.timeout
        )

        acceptance = self._goal_acceptance(
            goal.acceptance_radius
        )

        if not self._ensure_offboard_and_armed(
            timeout_s
        ):

            goal_handle.abort()

            return self._make_result(
                False,
                'Failed to enter OFFBOARD/ARM: '
                f'{self._last_control_error}',
            )

        current = self._get_current_enu()

        yaw = self._resolve_goal_yaw(
            goal.yaw
        )

        target = [
            current[0],
            current[1],
            altitude,
        ]

        self.get_logger().info(
            'TAKEOFF target (ENU): '
            f'x={target[0]:.2f}, '
            f'y={target[1]:.2f}, '
            f'z={target[2]:.2f}'
        )

        self._set_target(
            target,
            yaw,
        )

        return self._wait_until_target(
            goal_handle=goal_handle,
            target=target,
            acceptance=acceptance,
            timeout_s=timeout_s,
            state='TAKEOFF',
            success_message=(
                f'Takeoff reached '
                f'{altitude:.2f} m.'
            ),
        )

    # =========================================================================
    # GOTO
    # =========================================================================

    def _execute_goto(
        self,
        goal_handle,
    ):

        goal = goal_handle.request

        timeout_s = self._goal_timeout(
            goal.timeout
        )

        acceptance = self._goal_acceptance(
            goal.acceptance_radius
        )

        yaw = self._resolve_goal_yaw(
            goal.yaw
        )

        target = [
            float(goal.x),
            float(goal.y),
            float(goal.z),
        ]

        if not self._ensure_offboard_and_armed(
            timeout_s
        ):

            goal_handle.abort()

            return self._make_result(
                False,
                'Failed to enter OFFBOARD/ARM: '
                f'{self._last_control_error}',
            )

        self.get_logger().info(
            'GOTO target (ENU): '
            f'x={target[0]:.2f}, '
            f'y={target[1]:.2f}, '
            f'z={target[2]:.2f}'
        )

        self._set_target(
            target,
            yaw,
        )

        return self._wait_until_target(
            goal_handle=goal_handle,
            target=target,
            acceptance=acceptance,
            timeout_s=timeout_s,
            state='GOTO',
            success_message='GOTO target reached.',
        )

    # =========================================================================
    # HOVER
    # =========================================================================

    def _execute_hover(
        self,
        goal_handle,
    ):

        goal = goal_handle.request

        timeout_s = self._goal_timeout(
            goal.timeout
        )

        duration_s = (
            float(goal.duration)
            if goal.duration > 0.0
            else self.DEFAULT_HOVER_DURATION_S
        )

        target = self._get_current_enu()
        yaw = self._get_current_yaw_enu()

        if not self._ensure_offboard_and_armed(
            timeout_s
        ):

            goal_handle.abort()

            return self._make_result(
                False,
                'Failed to maintain OFFBOARD/ARM: '
                f'{self._last_control_error}',
            )

        self._set_target(
            target,
            yaw,
        )

        start = time.monotonic()

        while (
            time.monotonic() - start
            < duration_s
        ):

            if self._handle_stop_request(
                goal_handle
            ):

                return self._make_result(
                    False,
                    'HOVER interrupted.',
                )

            self._publish_feedback(
                goal_handle,
                'HOVER',
                target,
            )

            time.sleep(0.10)

        goal_handle.succeed()

        return self._make_result(
            True,
            f'Hover completed for '
            f'{duration_s:.1f} s.',
        )

    # =========================================================================
    # RETURN HOME
    # =========================================================================

    def _execute_return_home(
        self,
        goal_handle,
    ):

        goal = goal_handle.request

        with self._state_lock:

            home = (
                None
                if self._home_enu is None
                else list(self._home_enu)
            )

        if home is None:

            goal_handle.abort()

            return self._make_result(
                False,
                'Local home is unavailable.',
            )

        current = self._get_current_enu()

        altitude = (
            float(goal.z)
            if goal.z > 0.0
            else current[2]
        )

        target = [
            home[0],
            home[1],
            altitude,
        ]

        timeout_s = self._goal_timeout(
            goal.timeout
        )

        acceptance = self._goal_acceptance(
            goal.acceptance_radius
        )

        yaw = self._resolve_goal_yaw(
            goal.yaw
        )

        if not self._ensure_offboard_and_armed(
            timeout_s
        ):

            goal_handle.abort()

            return self._make_result(
                False,
                'Failed to maintain OFFBOARD/ARM: '
                f'{self._last_control_error}',
            )

        self._set_target(
            target,
            yaw,
        )

        return self._wait_until_target(
            goal_handle=goal_handle,
            target=target,
            acceptance=acceptance,
            timeout_s=timeout_s,
            state='RETURN_HOME',
            success_message=(
                'Returned to local home x/y.'
            ),
        )

    # =========================================================================
    # LAND
    # =========================================================================

    def _execute_land(
        self,
        goal_handle,
    ):

        timeout_s = self._goal_timeout(
            goal_handle.request.timeout
        )

        self._stream_offboard = False

        self.get_logger().info(
            'Requesting PX4 NAV_LAND.'
        )

        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )

        deadline = (
            time.monotonic()
            + timeout_s
        )

        while (
            time.monotonic()
            < deadline
        ):

            if goal_handle.is_cancel_requested:

                self._hold_current_position()

                goal_handle.canceled()

                return self._make_result(
                    False,
                    'LAND canceled; '
                    'safe hold requested.',
                )

            land_ack = self._get_recent_ack(
                VehicleCommand.VEHICLE_CMD_NAV_LAND
            )

            if self._ack_is_failure(
                land_ack
            ):

                goal_handle.abort()

                return self._make_result(
                    False,
                    'PX4 rejected NAV_LAND: '
                    f'{self._format_ack(land_ack)}',
                )

            self._publish_feedback(
                goal_handle,
                'LAND',
                self._get_current_enu(),
            )

            if not self._is_armed():

                goal_handle.succeed()

                return self._make_result(
                    True,
                    'Landing complete; '
                    'vehicle disarmed.',
                )

            time.sleep(0.10)

        goal_handle.abort()

        return self._make_result(
            False,
            'LAND timed out before disarm.',
        )

    # =========================================================================
    # ABORT
    # =========================================================================

    def _execute_abort(
        self,
        goal_handle,
    ):

        self._abort_requested = True

        if not self._has_valid_position():

            goal_handle.abort()

            return self._make_result(
                False,
                'ABORT received but '
                'local position is invalid.',
            )

        self._hold_current_position()

        goal_handle.succeed()

        return self._make_result(
            True,
            'ABORT accepted; '
            'safe-hold setpoint latched.',
        )

    # =========================================================================
    # Target waiting
    # =========================================================================

    def _wait_until_target(
        self,
        goal_handle,
        target,
        acceptance,
        timeout_s,
        state,
        success_message,
    ):

        deadline = (
            time.monotonic()
            + timeout_s
        )

        while (
            time.monotonic()
            < deadline
        ):

            if self._handle_stop_request(
                goal_handle
            ):

                return self._make_result(
                    False,
                    f'{state} interrupted.',
                )

            self._publish_feedback(
                goal_handle,
                state,
                target,
            )

            distance = self._distance_to(
                target
            )

            if distance <= acceptance:

                goal_handle.succeed()

                return self._make_result(
                    True,
                    success_message,
                )

            if not self._is_armed():

                self._hold_current_position()

                goal_handle.abort()

                return self._make_result(
                    False,
                    f'{state} failed: '
                    'vehicle became disarmed.',
                )

            if not self._is_offboard():

                self._hold_current_position()

                goal_handle.abort()

                return self._make_result(
                    False,
                    f'{state} failed: '
                    'PX4 left OFFBOARD mode.',
                )

            time.sleep(0.10)

        self._hold_current_position()

        goal_handle.abort()

        return self._make_result(
            False,
            f'{state} timed out; '
            'safe hold activated.',
        )

    # =========================================================================
    # Stop / cancel
    # =========================================================================

    def _handle_stop_request(
        self,
        goal_handle,
    ):

        if goal_handle.is_cancel_requested:

            self._hold_current_position()

            goal_handle.canceled()

            return True

        if self._abort_requested:

            self._hold_current_position()

            goal_handle.abort()

            return True

        return False

    # =========================================================================
    # Feedback / result
    # =========================================================================

    def _publish_feedback(
        self,
        goal_handle,
        state,
        target,
    ):

        current = self._get_current_enu()

        feedback = (
            ExecuteFlightAction.Feedback()
        )

        feedback.state = str(state)

        feedback.current_x = float(
            current[0]
        )

        feedback.current_y = float(
            current[1]
        )

        feedback.current_z = float(
            current[2]
        )

        feedback.distance_to_target = float(
            self._distance_to(target)
        )

        feedback.armed = bool(
            self._is_armed()
        )

        feedback.offboard = bool(
            self._is_offboard()
        )

        goal_handle.publish_feedback(
            feedback
        )

    def _make_result(
        self,
        success,
        message,
    ):

        current = self._get_current_enu()

        result = ExecuteFlightAction.Result()

        result.success = bool(success)
        result.message = str(message)

        result.final_x = float(
            current[0]
        )

        result.final_y = float(
            current[1]
        )

        result.final_z = float(
            current[2]
        )

        return result

    # =========================================================================
    # State helpers
    # =========================================================================

    def _wait_for_position(
        self,
        timeout_s,
    ):

        deadline = (
            time.monotonic()
            + timeout_s
        )

        while (
            time.monotonic()
            < deadline
        ):

            if self._has_valid_position():
                return True

            time.sleep(0.10)

        return self._has_valid_position()

    def _hold_current_position(self):

        if not self._has_valid_position():
            return

        self._set_target(
            self._get_current_enu(),
            self._get_current_yaw_enu(),
        )

        self._stream_offboard = True

    def _set_target(
        self,
        target,
        yaw,
    ):

        with self._state_lock:

            self._target_enu = [
                float(target[0]),
                float(target[1]),
                float(target[2]),
            ]

            self._target_yaw_enu = float(
                yaw
            )

    def _get_current_enu(self):

        with self._state_lock:
            return list(
                self._current_enu
            )

    def _get_current_yaw_enu(self):

        with self._state_lock:
            return float(
                self._current_yaw_enu
            )

    def _has_valid_position(self):

        with self._state_lock:
            return bool(
                self._position_valid
            )

    def _preflight_checks_pass(self):

        with self._state_lock:

            if self._vehicle_status is None:
                return False

            return bool(
                self._vehicle_status
                .pre_flight_checks_pass
            )

    def _is_armed(self):

        with self._state_lock:

            if self._vehicle_status is None:
                return False

            return bool(
                self._vehicle_status.arming_state
                == VehicleStatus.ARMING_STATE_ARMED
            )

    def _is_offboard(self):

        with self._state_lock:

            if self._vehicle_status is None:
                return False

            return bool(
                self._vehicle_status.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            )

    # =========================================================================
    # ACK helpers
    # =========================================================================

    def _get_recent_ack(
        self,
        command,
    ):

        with self._state_lock:

            item = (
                self._last_ack_by_command.get(
                    int(command)
                )
            )

        if item is None:
            return None

        result, p1, p2, timestamp = item

        if (
            time.monotonic()
            - timestamp
            > 5.0
        ):

            return None

        return (
            result,
            p1,
            p2,
        )

    def _ack_is_failure(
        self,
        ack,
    ):

        if ack is None:
            return False

        result = ack[0]

        return result in (
            VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED,
        )

    def _format_ack(
        self,
        ack,
    ):

        if ack is None:
            return 'no ACK received'

        result, p1, p2 = ack

        names = {
            VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                'ACCEPTED',

            VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
                'TEMPORARILY_REJECTED',

            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED:
                'DENIED',

            VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED:
                'UNSUPPORTED',

            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED:
                'FAILED',

            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS:
                'IN_PROGRESS',

            VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED:
                'CANCELLED',
        }

        result_name = names.get(
            result,
            f'UNKNOWN({result})',
        )

        return (
            f'result={result_name}, '
            f'result_param1={p1}, '
            f'result_param2={p2}'
        )

    # =========================================================================
    # Geometry / goal helpers
    # =========================================================================

    def _distance_to(
        self,
        target,
    ):

        return float(
            math.dist(
                self._get_current_enu(),
                target,
            )
        )

    def _resolve_goal_yaw(
        self,
        yaw,
    ):

        if math.isfinite(
            float(yaw)
        ):

            return wrap_pi(
                float(yaw)
            )

        return self._get_current_yaw_enu()

    def _goal_timeout(
        self,
        requested,
    ):

        if requested > 0.0:
            return float(requested)

        return self.DEFAULT_TIMEOUT_S

    def _goal_acceptance(
        self,
        requested,
    ):

        if requested > 0.0:
            return float(requested)

        return self.DEFAULT_ACCEPTANCE_RADIUS_M

    # =========================================================================
    # Time
    # =========================================================================

    def _now_us(self):

        return int(
            self.get_clock()
            .now()
            .nanoseconds
            // 1000
        )

    # =========================================================================
    # Shutdown
    # =========================================================================

    def destroy_node(self):

        self._stream_offboard = False

        self._action_server.destroy()

        super().destroy_node()


# =============================================================================
# Main
# =============================================================================


def main(args=None):

    rclpy.init(args=args)

    node = Px4Executor()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(
        node
    )

    try:

        executor.spin()

    except KeyboardInterrupt:

        pass

    finally:

        executor.shutdown()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
