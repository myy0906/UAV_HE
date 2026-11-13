import json
import os
import queue
import signal
import subprocess
import threading
import time

from datetime import datetime
from pathlib import Path


import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from uav_vln_interfaces.action import ExecuteMission


class MissionConsoleNode(Node):

    ROSBAG_TOPICS = [
        # ------------------------------------------------------
        # High-level UAV state
        # ------------------------------------------------------
        '/uav/vehicle_state',

        # ------------------------------------------------------
        # Perception and target localization
        # ------------------------------------------------------
        '/uav/perception/target_detection',
        '/uav/mapping/semantic_target',

        # ------------------------------------------------------
        # Obstacle avoidance and path planning
        # ------------------------------------------------------
        '/uav/navigation/occupancy_grid',
        '/uav/navigation/planned_path',

        # ------------------------------------------------------
        # PX4 commands sent by our executor
        # ------------------------------------------------------
        '/fmu/in/offboard_control_mode',
        '/fmu/in/trajectory_setpoint',
        '/fmu/in/vehicle_command',

        # ------------------------------------------------------
        # PX4 state feedback
        # ------------------------------------------------------
        '/fmu/out/vehicle_local_position_v1',
        '/fmu/out/vehicle_status_v1',
        '/fmu/out/vehicle_command_ack',
    ]

    def __init__(self, ui_queue):

        super().__init__(
            'mission_console_node'
        )

        self.ui_queue = ui_queue

        self.client = ActionClient(
            self,
            ExecuteMission,
            '/mission/execute',
        )

        self.goal_handle = None

        # ======================================================
        # Experiment logging
        # ======================================================

        self.log_root = (
            Path.home()
            / 'px4_ros2_ws'
            / 'logs'
        )

        self.log_root.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.run_dir = None
        self.bag_process = None
        self.bag_log_file = None

        self.metadata = {}
        self.file_lock = threading.Lock()

        self.emit_log(
            'Mission Console ROS node started.'
        )

        self.emit_log(
            'Automatic per-mission logging enabled.'
        )

        self.emit_log(
            f'Log root: {self.log_root}'
        )

    # ==========================================================
    # Logging helpers
    # ==========================================================

    def emit_log(self, text):

        text = str(text)

        self.ui_queue.put(
            (
                'log',
                text,
            )
        )

        if self.run_dir is None:
            return

        with self.file_lock:

            path = (
                self.run_dir
                / 'mission.log'
            )

            timestamp = (
                datetime.now()
                .astimezone()
                .isoformat(
                    timespec='milliseconds'
                )
            )

            with path.open(
                'a',
                encoding='utf-8',
            ) as file:

                file.write(
                    f'[{timestamp}] {text}\n'
                )

    def write_metadata(self):

        if self.run_dir is None:
            return

        with self.file_lock:

            path = (
                self.run_dir
                / 'mission.json'
            )

            with path.open(
                'w',
                encoding='utf-8',
            ) as file:

                json.dump(
                    self.metadata,
                    file,
                    ensure_ascii=False,
                    indent=2,
                )

    def create_experiment(
        self,
        instruction,
    ):

        timestamp = datetime.now().strftime(
            '%Y%m%d_%H%M%S_%f'
        )

        self.run_dir = (
            self.log_root
            / timestamp
        )

        self.run_dir.mkdir(
            parents=True,
            exist_ok=False,
        )

        self.metadata = {
            'experiment_id':
                timestamp,

            'start_time':
                datetime.now()
                .astimezone()
                .isoformat(),

            'end_time':
                None,

            'instruction':
                instruction,

            'status':
                'STARTING',

            'success':
                None,

            'result_message':
                None,

            'parsed_json':
                None,

            'rosbag_directory':
                str(
                    self.run_dir
                    / 'rosbag'
                ),

            'rosbag_topics':
                list(
                    self.ROSBAG_TOPICS
                ),

            'px4_ulog_path':
                None,
        }

        self.write_metadata()

        self.emit_log(
            '========================================'
        )

        self.emit_log(
            f'Experiment directory: {self.run_dir}'
        )

        self.emit_log(
            'Natural-language instruction:'
        )

        self.emit_log(
            instruction
        )

    def start_rosbag(self):

        if self.run_dir is None:

            raise RuntimeError(
                'Experiment directory '
                'has not been created.'
            )

        bag_dir = (
            self.run_dir
            / 'rosbag'
        )

        bag_log_path = (
            self.run_dir
            / 'rosbag_record.log'
        )

        self.bag_log_file = (
            bag_log_path.open(
                'w',
                encoding='utf-8',
            )
        )

        command = [
            'ros2',
            'bag',
            'record',

            '-o',
            str(
                bag_dir
            ),
        ]

        command.extend(
            self.ROSBAG_TOPICS
        )

        self.emit_log(
            'Starting ROS 2 bag recording...'
        )

        self.bag_process = subprocess.Popen(
            command,

            stdout=self.bag_log_file,
            stderr=subprocess.STDOUT,

            start_new_session=True,
        )

        # Give rosbag a moment to initialise.
        time.sleep(
            0.8
        )

        if (
            self.bag_process.poll()
            is not None
        ):

            raise RuntimeError(
                'ros2 bag record exited '
                'immediately. Check '
                'rosbag_record.log.'
            )

        self.emit_log(
            'ROS 2 bag recording started.'
        )

    def stop_rosbag(self):

        process = self.bag_process

        if process is None:
            return

        self.emit_log(
            'Stopping ROS 2 bag recording...'
        )

        try:

            if process.poll() is None:

                os.killpg(
                    process.pid,
                    signal.SIGINT,
                )

                try:

                    process.wait(
                        timeout=15.0
                    )

                except subprocess.TimeoutExpired:

                    self.emit_log(
                        'rosbag did not stop '
                        'after SIGINT; sending SIGTERM.'
                    )

                    os.killpg(
                        process.pid,
                        signal.SIGTERM,
                    )

                    try:

                        process.wait(
                            timeout=5.0
                        )

                    except subprocess.TimeoutExpired:

                        self.emit_log(
                            'rosbag still running; '
                            'sending SIGKILL.'
                        )

                        os.killpg(
                            process.pid,
                            signal.SIGKILL,
                        )

                        process.wait(
                            timeout=3.0
                        )

        except ProcessLookupError:
            pass

        except Exception as exc:

            self.emit_log(
                f'Error while stopping rosbag: {exc}'
            )

        finally:

            self.bag_process = None

            if self.bag_log_file is not None:

                self.bag_log_file.flush()
                self.bag_log_file.close()

                self.bag_log_file = None

        self.emit_log(
            'ROS 2 bag recording stopped.'
        )

    # ==========================================================
    # PX4 ULog association
    # ==========================================================

    @staticmethod
    def find_latest_ulog():

        root = (
            Path.home()
            / 'PX4-Autopilot'
            / 'build'
            / 'px4_sitl_default'
        )

        if not root.exists():
            return None

        candidates = []

        try:

            for path in root.rglob(
                '*.ulg'
            ):

                try:

                    candidates.append(
                        (
                            path.stat().st_mtime,
                            path,
                        )
                    )

                except OSError:
                    pass

        except OSError:
            return None

        if not candidates:
            return None

        candidates.sort(
            key=lambda item: item[0]
        )

        return str(
            candidates[-1][1]
        )

    # ==========================================================
    # Mission submission
    # ==========================================================

    def send_instruction(
        self,
        instruction,
    ):

        instruction = (
            instruction.strip()
        )

        if not instruction:

            self.ui_queue.put(
                (
                    'log',
                    '请输入任务指令。',
                )
            )

            self.ui_queue.put(
                (
                    'busy',
                    False,
                )
            )

            return

        if not self.client.server_is_ready():

            self.ui_queue.put(
                (
                    'log',
                    'Mission Action Server 尚未就绪。',
                )
            )

            self.ui_queue.put(
                (
                    'busy',
                    False,
                )
            )

            return

        try:

            self.create_experiment(
                instruction
            )

            self.start_rosbag()

        except Exception as exc:

            self.emit_log(
                f'创建实验日志失败：{exc}'
            )

            self.stop_rosbag()

            self.metadata[
                'status'
            ] = 'LOGGING_FAILED'

            self.metadata[
                'success'
            ] = False

            self.metadata[
                'result_message'
            ] = str(exc)

            self.metadata[
                'end_time'
            ] = (
                datetime.now()
                .astimezone()
                .isoformat()
            )

            self.write_metadata()

            self.ui_queue.put(
                (
                    'busy',
                    False,
                )
            )

            return

        self.metadata[
            'status'
        ] = 'MISSION_SENT'

        self.write_metadata()

        goal = ExecuteMission.Goal()

        goal.instruction = (
            instruction
        )

        self.emit_log(
            'Sending mission to Mission Manager...'
        )

        future = (
            self.client
            .send_goal_async(
                goal,

                feedback_callback=
                    self.feedback_callback,
            )
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(
        self,
        future,
    ):

        try:

            goal_handle = (
                future.result()
            )

        except Exception as exc:

            self.emit_log(
                f'发送任务失败：{exc}'
            )

            self.finish_experiment(
                success=False,
                status='SEND_FAILED',
                message=str(exc),
                parsed_json=None,
            )

            self.ui_queue.put(
                (
                    'busy',
                    False,
                )
            )

            return

        if not goal_handle.accepted:

            self.emit_log(
                '任务被 Mission Manager 拒绝。'
            )

            self.finish_experiment(
                success=False,
                status='REJECTED',
                message=(
                    'Mission Manager '
                    'rejected the goal.'
                ),
                parsed_json=None,
            )

            self.ui_queue.put(
                (
                    'busy',
                    False,
                )
            )

            return

        self.goal_handle = (
            goal_handle
        )

        self.metadata[
            'status'
        ] = 'RUNNING'

        self.write_metadata()

        self.emit_log(
            'Mission accepted.'
        )

        result_future = (
            goal_handle
            .get_result_async()
        )

        result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(
        self,
        feedback_message,
    ):

        feedback = (
            feedback_message.feedback
        )

        progress_percent = (
            100.0
            * float(
                feedback.progress
            )
        )

        text = (
            f'[{feedback.stage}] '
            f'{feedback.detail} '
            f'({progress_percent:.0f}%)'
        )

        # Save stage in mission.log.
        self.emit_log(
            text
        )

        # Update GUI status without
        # duplicating log text there.
        self.ui_queue.put(
            (
                'stage',
                feedback.stage,
            )
        )

    def result_callback(
        self,
        future,
    ):

        parsed_json = None

        try:

            wrapped = (
                future.result()
            )

            result = (
                wrapped.result
            )

            parsed_json = (
                result.parsed_json
            )

            if result.success:

                self.emit_log(
                    'Mission completed successfully.'
                )

                status = 'SUCCEEDED'

            else:

                self.emit_log(
                    'Mission failed.'
                )

                status = 'FAILED'

            self.emit_log(
                'Result: '
                + str(
                    result.message
                )
            )

            if parsed_json:

                self.emit_log(
                    'Qwen parsed mission: '
                    + parsed_json
                )

            self.finish_experiment(
                success=bool(
                    result.success
                ),

                status=status,

                message=str(
                    result.message
                ),

                parsed_json=
                    parsed_json,
            )

        except Exception as exc:

            self.emit_log(
                f'读取任务结果失败：{exc}'
            )

            self.finish_experiment(
                success=False,
                status='RESULT_ERROR',
                message=str(exc),
                parsed_json=parsed_json,
            )

        finally:

            self.goal_handle = None

            self.ui_queue.put(
                (
                    'busy',
                    False,
                )
            )

    def finish_experiment(
        self,
        success,
        status,
        message,
        parsed_json,
    ):

        if self.run_dir is None:
            return

        self.metadata[
            'status'
        ] = status

        self.metadata[
            'success'
        ] = bool(
            success
        )

        self.metadata[
            'result_message'
        ] = str(
            message
        )

        self.metadata[
            'parsed_json'
        ] = (
            parsed_json
        )

        self.metadata[
            'end_time'
        ] = (
            datetime.now()
            .astimezone()
            .isoformat()
        )

        latest_ulog = (
            self.find_latest_ulog()
        )

        self.metadata[
            'px4_ulog_path'
        ] = latest_ulog

        self.write_metadata()

        self.stop_rosbag()

        self.emit_log(
            'Experiment saved.'
        )

        self.emit_log(
            f'Experiment directory: {self.run_dir}'
        )

        if latest_ulog:

            self.emit_log(
                'Associated PX4 ULog: '
                + latest_ulog
            )

        else:

            self.emit_log(
                'No PX4 ULog was found.'
            )

        self.write_metadata()

        # The completed experiment remains
        # accessible, but the next mission
        # will create a new directory.
        self.run_dir = None

    # ==========================================================
    # Mission cancellation
    # ==========================================================

    def cancel_current(
        self,
    ):

        if self.goal_handle is None:

            self.ui_queue.put(
                (
                    'log',
                    '当前没有正在执行的任务。',
                )
            )

            return

        self.emit_log(
            'Requesting mission cancellation...'
        )

        future = (
            self.goal_handle
            .cancel_goal_async()
        )

        future.add_done_callback(
            self.cancel_response_callback
        )

    def cancel_response_callback(
        self,
        future,
    ):

        try:

            response = (
                future.result()
            )

            if len(
                response.goals_canceling
            ) > 0:

                self.emit_log(
                    'Cancellation request accepted.'
                )

            else:

                self.emit_log(
                    'Mission Manager did not '
                    'accept cancellation request.'
                )

        except Exception as exc:

            self.emit_log(
                f'取消任务失败：{exc}'
            )

    def shutdown_logging(self):

        if self.bag_process is None:
            return

        self.emit_log(
            'Mission Console shutting down.'
        )

        self.metadata[
            'status'
        ] = 'CONSOLE_CLOSED'

        self.metadata[
            'success'
        ] = False

        self.metadata[
            'end_time'
        ] = (
            datetime.now()
            .astimezone()
            .isoformat()
        )

        self.metadata[
            'px4_ulog_path'
        ] = (
            self.find_latest_ulog()
        )

        self.write_metadata()

        self.stop_rosbag()




# ==========================================================
# Command-line console for the known-map system
# ==========================================================

import shutil


class CommandLineNode(
    MissionConsoleNode
):

    def create_experiment(
        self,
        instruction,
    ):

        super().create_experiment(
            instruction
        )

        # Save the simulation truth only
        # for post-flight evaluation.
        # The mission manager never reads it.
        scene = (

            Path.home()

            / "px4_ros2_ws"

            / "src/uav_vln_bringup"

            / "models/vln_scene/model.sdf"
        )

        if scene.is_file():

            shutil.copy2(

                scene,

                self.run_dir
                / "scene_truth.sdf",
            )

    def finish_experiment(

        self,

        success,

        status,

        message,

        parsed_json,
    ):

        log_dir = self.run_dir

        result_dir = None

        if parsed_json:

            try:

                envelope = json.loads(
                    parsed_json
                )

                result_dir = Path(

                    envelope[
                        "result_directory"
                    ]
                )

                root = (

                    Path.home()

                    / "px4_ros2_ws"

                    / "results"
                ).resolve()

                result_dir = (
                    result_dir.resolve()
                )

                if not (

                    result_dir.parent
                    == root

                    and result_dir.is_dir()
                ):

                    result_dir = None

                if result_dir is not None:

                    self.metadata[
                        "result_directory"
                    ] = str(
                        result_dir
                    )

                    if log_dir is not None:

                        truth = (

                            log_dir

                            / "scene_truth.sdf"
                        )

                        if truth.is_file():

                            shutil.copy2(

                                truth,

                                result_dir

                                / "scene_truth.sdf",
                            )

                        (
                            result_dir

                            / "rosbag_location.txt"
                        ).write_text(

                            str(

                                log_dir
                                / "rosbag"
                            ),

                            encoding="utf-8",
                        )

            except Exception as exc:

                self.emit_log(

                    "Evaluation link error: "

                    + str(exc)
                )

        super().finish_experiment(

            success,

            status,

            message,

            parsed_json,
        )

        if result_dir is not None:

            self.ui_queue.put(

                (

                    "log",

                    "Evaluation data: "

                    + str(result_dir),
                )
            )


def print_help():

    print(
        "\nAvailable commands:"
    )

    print(
        "  /help    Show this help"
    )

    print(
        "  /status  Show mission status"
    )

    print(
        "  /cancel  Cancel active mission"
    )

    print(
        "  /exit    Exit while idle"
    )

    print(
        "\nOr enter a natural-language "
        "inspection mission.\n"
    )


def input_worker(command_queue):

    while True:

        try:

            text = input(
                "\n>>> "
            ).strip()

        except EOFError:

            command_queue.put(
                "/exit"
            )

            return

        if text:

            command_queue.put(
                text
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    messages = queue.Queue()

    commands = queue.Queue()

    node = CommandLineNode(
        messages
    )

    executor = (
        MultiThreadedExecutor(
            num_threads=2
        )
    )

    executor.add_node(node)

    worker = threading.Thread(

        target=executor.spin,

        daemon=True,
    )

    worker.start()

    thread = threading.Thread(

        target=input_worker,

        args=(commands,),

        daemon=True,
    )

    print(
        "\n"
        "====================================\n"
        "UAV-VLN Known-map Mission CLI\n"
        "===================================="
    )

    print_help()

    thread.start()

    busy = False

    exiting = False

    try:

        while (

            rclpy.ok()

            and not exiting
        ):

            # Process ROS feedback and status.
            while True:

                try:

                    event, value = (
                        messages.get_nowait()
                    )

                except queue.Empty:

                    break

                if event == "busy":

                    busy = bool(value)

                elif event == "log":

                    print(

                        "\n" + str(value),

                        flush=True,
                    )

                else:

                    print(

                        f"\n[{event}] {value}",

                        flush=True,
                    )

            try:

                command = commands.get(
                    timeout=0.1
                )

            except queue.Empty:

                continue

            if command == "/help":

                print_help()

            elif command == "/status":

                print(

                    "\nMission status:",

                    "BUSY"
                    if busy
                    else "IDLE",
                )

                print(

                    "Action server:",

                    node.client.server_is_ready(),
                )

            elif command == "/cancel":

                node.cancel_current()

            elif command == "/exit":

                if busy:

                    print(

                        "\nMission active. "

                        "Use /cancel and wait "

                        "for recovery to finish."
                    )

                else:

                    exiting = True

            elif command.startswith("/"):

                print(
                    "Unknown CLI command"
                )

            elif busy:

                print(

                    "A mission is already active. "

                    "Use /status or /cancel."
                )

            else:

                busy = True

                node.send_instruction(
                    command
                )

    except KeyboardInterrupt:

        if busy:

            print(

                "\nCanceling active mission. "

                "Keep PX4 and Mission Manager "

                "running."
            )

            # If goal acceptance is still
            # pending, wait briefly for it.
            deadline = (

                time.monotonic()

                + 90.0
            )

            cancel_sent = False

            while (

                time.monotonic()
                < deadline
            ):

                if (

                    not cancel_sent

                    and node.goal_handle
                    is not None
                ):

                    node.cancel_current()

                    cancel_sent = True

                try:

                    event, value = messages.get(
                        timeout=0.2
                    )

                    if (

                        event == "busy"

                        and not value
                    ):

                        busy = False

                        break

                    if event == "log":

                        print(value)

                except queue.Empty:

                    pass

            if busy:

                print(

                    "Mission termination "

                    "not confirmed. "

                    "Inspect PX4 before "

                    "sending another task."
                )

    finally:

        node.shutdown_logging()

        executor.shutdown()

        worker.join(
            timeout=3.0
        )

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
