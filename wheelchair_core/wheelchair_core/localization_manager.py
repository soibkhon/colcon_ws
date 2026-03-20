import math
import os
import subprocess
import threading
import time
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration


def _resolve_share_file(package_name: str, relative_path: str) -> str:
    return os.path.join(get_package_share_directory(package_name), relative_path)


def _bool_to_launch(value: bool) -> str:
    return 'true' if value else 'false'


def _absolute_topic(name: str) -> str:
    if not name.startswith('/'):
        return '/' + name
    return name


class LocalizationManager(Node):
    def __init__(self) -> None:
        super().__init__('localization_manager')

        def declare(name: str, default):
            try:
                self.declare_parameter(name, default)
            except ParameterAlreadyDeclaredException:
                pass
            return self.get_parameter(name).get_parameter_value()

        self.use_sim_time: bool = declare('use_sim_time', False).bool_value
        self.map_pcd_path: str = os.path.expanduser(declare('map_pcd_path', os.path.join(os.path.expanduser('~'), 'colcon_ws', 'src', 'FAST_LIO', 'PCD', 'clear.pcd')).string_value)
        self.single_lidar_bd_code: str = declare('single_lidar_bd_code', 'livox0000000001').string_value
        self.single_lidar_topic: str = declare('single_lidar_topic', 'livox/lidar_192_168_1_172').string_value
        self.single_imu_topic: str = declare('single_imu_topic', 'livox/imu_192_168_1_172').string_value
        self.localization_launch_path: str = declare('localization_launch', _resolve_share_file('wheelchair_core', 'launch/wheelchair_localization_phase.launch.py')).string_value
        self.navigation_launch_path: str = declare('navigation_launch', _resolve_share_file('wheelchair_core', 'launch/wheelchair_navigation.launch.py')).string_value
        self.localization_timeout_sec: float = declare('localization_timeout_sec', 30.0).double_value
        self.localization_required_messages: int = declare('localization_required_messages', 40).integer_value
        self.post_localization_lidar_delay_sec: float = declare('post_localization_lidar_delay_sec', 3.0).double_value

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, self.use_sim_time)])

        if not os.path.isabs(self.map_pcd_path):
            self.map_pcd_path = os.path.abspath(self.map_pcd_path)
        if not os.path.exists(self.map_pcd_path):
            self.get_logger().warn(f"PCD map '{self.map_pcd_path}' not found; FAST-LIO will fallback to mapping mode.")

        self._localization_event = threading.Event()
        self._localization_active = False
        self._localization_message_count = 0
        self._sequence_lock = threading.Lock()
        self._sequence_thread: Optional[threading.Thread] = None

        self._localization_process: Optional[subprocess.Popen] = None
        self._navigation_process: Optional[subprocess.Popen] = None

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=300))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)
        self._map_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._latest_pose: Optional[TransformStamped] = None
        self._map_to_base_timer = None

        self._odom_sub = self.create_subscription(
            Odometry,
            '/fastlio_odom',
            self._odom_callback,
            10
        )

        self._trigger_srv = self.create_service(Trigger, 'localize_once', self._handle_trigger)

        self._startup_timer = self.create_timer(0.5, self._startup_callback)

    def _startup_callback(self) -> None:
        self._startup_timer.cancel()
        if not self._start_sequence(reason='startup'):
            self.get_logger().warn('Localization sequence already running during startup.')

    def _handle_trigger(self, request, response):
        success = self._start_sequence(reason='manual')
        response.success = success
        response.message = 'Localization triggered.' if success else 'Localization already running.'
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
        self.get_logger().info(f'Starting localization sequence ({reason}).')
        try:
            self._stop_navigation()

            localized = self._run_localization()

            self._stop_localization()

            if not localized:
                self.get_logger().error('Localization failed or timed out; keeping navigation offline.')
                return

            time.sleep(self.post_localization_lidar_delay_sec)
            self._start_navigation()
            self.get_logger().info('Localization complete; navigation stack relaunched.')
        finally:
            with self._sequence_lock:
                self._sequence_thread = None

    def _run_localization(self) -> bool:
        self._localization_event.clear()
        self._localization_message_count = 0
        self._localization_active = True

        args = [
            f'use_sim_time:={_bool_to_launch(self.use_sim_time)}',
            f'pcd_map_path:={self.map_pcd_path}',
            f'bd_code:={self.single_lidar_bd_code}',
            f'lidar_topic:={_absolute_topic(self.single_lidar_topic)}',
            f'imu_topic:={_absolute_topic(self.single_imu_topic)}',
        ]

        self._localization_process = self._start_launch(
            'wheelchair_core',
            os.path.basename(self.localization_launch_path),
            args
        )

        if not self._localization_event.wait(timeout=self.localization_timeout_sec):
            self.get_logger().error('Timeout while waiting for FAST-LIO odometry during localization.')
            self._localization_active = False
            return False

        self._latest_pose = self._capture_pose()
        self.get_logger().info('Localization odometry received; proceeding to navigation restart.')
        self._localization_active = False
        return True

    def _start_navigation(self) -> None:
        if self._navigation_process:
            return

        args = [
            f'use_sim_time:={_bool_to_launch(self.use_sim_time)}'
        ]
        self._navigation_process = self._start_launch(
            'wheelchair_core',
            os.path.basename(self.navigation_launch_path),
            args
        )
        if self._latest_pose is not None:
            self.get_logger().info('Publishing initial pose from localization results.')
            if self._map_to_base_timer:
                self._map_to_base_timer.cancel()
            self._map_to_base_timer = self.create_timer(0.1, self._broadcast_map_pose)
            time.sleep(self.post_localization_lidar_delay_sec)
            self._publish_initial_pose()
        else:
            self.get_logger().warn('No pose captured from localization; initial pose not published.')

    def _stop_localization(self) -> None:
        self._stop_process(self._localization_process)
        self._localization_process = None
        self._localization_active = False

    def _stop_navigation(self) -> None:
        self._stop_process(self._navigation_process)
        self._navigation_process = None

    def _start_launch(self, package: str, launch_file: str, arguments: list[str]) -> subprocess.Popen:
        cmd = ['ros2', 'launch', package, launch_file]
        cmd.extend(arguments)
        self.get_logger().info(f'Starting process: {" ".join(cmd)}')
        return subprocess.Popen(cmd)

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
            self.get_logger().warning(f'Process pid={process.pid} did not exit in time; killing.')
            process.kill()
            process.wait(timeout=5.0)

    def _odom_callback(self, msg: Odometry) -> None:
        if not self._localization_active:
            return
        self._localization_message_count += 1
        if self._localization_message_count >= self.localization_required_messages:
            self._localization_event.set()

    def destroy_node(self):
        self._stop_localization()
        self._stop_navigation()
        if self._map_to_base_timer:
            self._map_to_base_timer.cancel()
        return super().destroy_node()

    def _capture_pose(self):
        target = 'camera_init'
        source = 'body'
        deadline = self.get_clock().now() + Duration(seconds=5.0)
        while self.get_clock().now() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.5))
                map_to_base = TransformStamped()
                map_to_base.header.frame_id = 'map'
                map_to_base.child_frame_id = 'base_link'
                map_to_base.header.stamp = self.get_clock().now().to_msg()
                map_to_base.transform.translation.x = transform.transform.translation.x
                map_to_base.transform.translation.y = transform.transform.translation.y
                map_to_base.transform.translation.z = transform.transform.translation.z
                map_to_base.transform.rotation = transform.transform.rotation
                self._flatten_transform(map_to_base)
                return map_to_base
            except tf2_ros.LookupException:
                time.sleep(0.1)
            except tf2_ros.ExtrapolationException:
                time.sleep(0.1)
            except tf2_ros.ConnectivityException:
                time.sleep(0.1)
        self.get_logger().error('Failed to capture transform camera_init -> body from FAST-LIO.')
        return None

    def _publish_initial_pose(self) -> None:
        if self._latest_pose is None:
            return
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = self._latest_pose.transform.translation.x
        pose_msg.pose.pose.position.y = self._latest_pose.transform.translation.y
        pose_msg.pose.pose.position.z = self._latest_pose.transform.translation.z
        pose_msg.pose.pose.orientation = self._latest_pose.transform.rotation
        for idx in (0, 7, 14, 21, 28, 35):
            pose_msg.pose.covariance[idx] = 0.05
        for _ in range(5):
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            self._initialpose_pub.publish(pose_msg)
            time.sleep(0.2)

    def _broadcast_map_pose(self) -> None:
        if self._latest_pose is None:
            return
        self._latest_pose.header.stamp = self.get_clock().now().to_msg()
        self._map_broadcaster.sendTransform(self._latest_pose)

    def _flatten_transform(self, transform: TransformStamped) -> None:
        transform.transform.translation.z = 0.0
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        half_yaw = yaw * 0.5
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(half_yaw)
        transform.transform.rotation.w = math.cos(half_yaw)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationManager()
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
