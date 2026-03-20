import math
import os
import subprocess
import threading
import time
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.duration import Duration
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from std_srvs.srv import Trigger
import tf2_ros


def _resolve_share_file(package_name: str, relative_path: str) -> str:
    return os.path.join(get_package_share_directory(package_name), relative_path)


def _bool_to_launch(value: bool) -> str:
    return 'true' if value else 'false'


class LocalizationManager2D(Node):
    def __init__(self) -> None:
        super().__init__('localization_manager_2d')

        def declare(name: str, default):
            try:
                self.declare_parameter(name, default)
            except ParameterAlreadyDeclaredException:
                pass
            return self.get_parameter(name).get_parameter_value()

        self.use_sim_time: bool = declare('use_sim_time', False).bool_value
        default_map = os.path.join(
            get_package_share_directory('wheelchair_core'),
            'maps',
            'clear_lab.yaml'
        )
        self.map_yaml: str = os.path.expanduser(declare('map_yaml', default_map).string_value)
        self.bd_code: str = declare('mid360_bd_code', 'livox0000000001').string_value
        self.cloud_topic: str = declare('cloud_topic', 'livox/lidar').string_value
        self.localization_launch: str = declare(
            'localization_launch',
            _resolve_share_file('wheelchair_core', 'launch/wheelchair_localization_2d.launch.py')
        ).string_value
        self.navigation_launch: str = declare(
            'navigation_launch',
            _resolve_share_file('wheelchair_core', 'launch/wheelchair_navigation.launch.py')
        ).string_value
        self.localization_timeout_sec: float = declare('localization_timeout_sec', 60.0).double_value
        self.post_localization_delay_sec: float = declare('post_localization_delay_sec', 3.0).double_value

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, self.use_sim_time)])

        self._sequence_lock = threading.Lock()
        self._sequence_thread: Optional[threading.Thread] = None

        self._localization_process: Optional[subprocess.Popen] = None
        self._navigation_process: Optional[subprocess.Popen] = None

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)
        self._map_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._latest_pose: Optional[TransformStamped] = None
        self._broadcast_timer = None

        self._trigger_srv = self.create_service(Trigger, 'localize_once', self._handle_trigger)
        self._startup_timer = self.create_timer(0.5, self._startup_callback)

    def _startup_callback(self) -> None:
        self._startup_timer.cancel()
        if not self._start_sequence('startup'):
            self.get_logger().warn('Localization already running at startup.')

    def _handle_trigger(self, request, response):
        success = self._start_sequence('manual')
        response.success = success
        response.message = 'Localization triggered.' if success else 'Localization already in progress.'
        return response

    def _start_sequence(self, reason: str) -> bool:
        with self._sequence_lock:
            if self._sequence_thread and self._sequence_thread.is_alive():
                return False
            self._sequence_thread = threading.Thread(
                target=self._run_sequence,
                args=(reason,),
                daemon=True
            )
            self._sequence_thread.start()
            return True

    def _run_sequence(self, reason: str) -> None:
        self.get_logger().info(f'Starting 2D localization sequence ({reason}).')
        try:
            self._stop_navigation()

            localized = self._run_localization()

            self._stop_localization()

            if not localized:
                self.get_logger().error('2D localization failed; navigation not restarted.')
                return

            self._start_navigation()
        finally:
            with self._sequence_lock:
                self._sequence_thread = None

    def _run_localization(self) -> bool:
        if not os.path.exists(self.map_yaml):
            self.get_logger().error(f'Map file {self.map_yaml} not found.')
            return False

        args = [
            f'use_sim_time:={_bool_to_launch(self.use_sim_time)}',
            f'map_file:={self.map_yaml}',
            f'bd_code:={self.bd_code}',
            f'cloud_topic:={self.cloud_topic}'
        ]
        self._localization_process = self._start_launch(self.localization_launch, args)

        deadline = self.get_clock().now() + Duration(seconds=self.localization_timeout_sec)
        while self.get_clock().now() < deadline:
            pose = self._capture_pose()
            if pose:
                self._latest_pose = pose
                self.get_logger().info('Localization pose captured.')
                return True
            time.sleep(0.1)

        self.get_logger().error('Timed out waiting for slam toolbox pose.')
        return False

    def _start_navigation(self) -> None:
        if self._navigation_process:
            return

        args = [
            f'use_sim_time:={_bool_to_launch(self.use_sim_time)}'
        ]
        self._navigation_process = self._start_launch(self.navigation_launch, args)

        if self._latest_pose is None:
            self.get_logger().warn('No pose to publish to navigation stack.')
            return

        if self._broadcast_timer:
            self._broadcast_timer.cancel()
        self._broadcast_timer = self.create_timer(0.1, self._broadcast_pose)
        time.sleep(self.post_localization_delay_sec)
        self._publish_initial_pose()

    def _start_launch(self, launch_path: str, arguments: list[str]) -> subprocess.Popen:
        cmd = ['ros2', 'launch', 'wheelchair_core', os.path.basename(launch_path)]
        cmd.extend(arguments)
        self.get_logger().info(f'Starting process: {" ".join(cmd)}')
        return subprocess.Popen(cmd)

    def _stop_localization(self) -> None:
        self._stop_process(self._localization_process)
        self._localization_process = None

    def _stop_navigation(self) -> None:
        self._stop_process(self._navigation_process)
        self._navigation_process = None
        if self._broadcast_timer:
            self._broadcast_timer.cancel()
            self._broadcast_timer = None

    def _stop_process(self, process: Optional[subprocess.Popen]) -> None:
        if process is None:
            return
        if process.poll() is not None:
            return
        self.get_logger().info(f'Stopping process pid={process.pid}')
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning(f'Process pid={process.pid} did not exit; killing.')
            process.kill()
            process.wait(timeout=5.0)

    def _capture_pose(self) -> Optional[TransformStamped]:
        try:
            transform = self._tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(),
                timeout=Duration(seconds=0.2)
            )
            self._flatten_transform(transform)
            return transform
        except tf2_ros.LookupException:
            return None
        except tf2_ros.ExtrapolationException:
            return None
        except tf2_ros.ConnectivityException:
            return None

    def _flatten_transform(self, transform: TransformStamped) -> None:
        transform.transform.translation.z = 0.0
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(yaw * 0.5)
        transform.transform.rotation.w = math.cos(yaw * 0.5)

    def _publish_initial_pose(self) -> None:
        if self._latest_pose is None:
            return
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = self._latest_pose.transform.translation.x
        pose_msg.pose.pose.position.y = self._latest_pose.transform.translation.y
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation = self._latest_pose.transform.rotation
        for idx in (0, 7, 14, 21, 28, 35):
            pose_msg.pose.covariance[idx] = 0.05
        for _ in range(5):
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            self._initialpose_pub.publish(pose_msg)
            time.sleep(0.1)

    def _broadcast_pose(self) -> None:
        if self._latest_pose is None:
            return
        self._latest_pose.header.stamp = self.get_clock().now().to_msg()
        self._map_broadcaster.sendTransform(self._latest_pose)

    def destroy_node(self):
        self._stop_localization()
        self._stop_navigation()
        if self._broadcast_timer:
            self._broadcast_timer.cancel()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationManager2D()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Localization manager interrupted; shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
