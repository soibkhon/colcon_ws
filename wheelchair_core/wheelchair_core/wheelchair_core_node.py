#!/usr/bin/env python3
"""
Wheelchair Core ROS2 Node
Integrates L2DB motor drivers with ROS2 diff_drive_controller
Publishes odometry and accepts velocity commands
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
import time
import threading
from wheelchair_core.l2db_motor_driver import L2DBMotorDriver


class WheelchairCoreNode(Node):
    """ROS2 node for wheelchair control using L2DB motor drivers"""
    
    def __init__(self):
        super().__init__('wheelchair_core_node')
        
        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('can_interface', 'socketcan'),
                ('can_channel', 'can0'),
                ('motor_left_id', 1),
                ('motor_right_id', 2),
                ('wheel_diameter', 0.243),  # 24.3cm wheels
                ('wheel_base', 0.598),       # 53cm wheelbase
                ('encoder_resolution', 4096),
                ('max_linear_velocity', 0.83),  # 3 km/h
                ('max_angular_velocity', 0.5),  # rad/s
                ('publish_rate', 50.0),  # Hz
                ('odom_frame_id', 'odom'),
                ('base_frame_id', 'base_link'),
                ('publish_tf', True),
                ('acceleration', 0.5),  # rps/s
                ('deceleration', 0.5),  # rps/s
                ('pose_covariance_diagonal', [0.01, 0.01, 0.001, 0.001, 0.001, 0.09]),
                ('twist_covariance_diagonal', [0.002, 0.001, 0.001, 0.001, 0.001, 0.02]),
            ]
        )
        
        # Get parameters
        self.can_interface = self.get_parameter('can_interface').value
        self.can_channel = self.get_parameter('can_channel').value
        self.motor_left_id = self.get_parameter('motor_left_id').value
        self.motor_right_id = self.get_parameter('motor_right_id').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.acceleration = self.get_parameter('acceleration').value
        self.deceleration = self.get_parameter('deceleration').value
        self.pose_covariance_diagonal = self.get_parameter('pose_covariance_diagonal').value
        self.twist_covariance_diagonal = self.get_parameter('twist_covariance_diagonal').value
        
        # Calculate conversion factors
        self.wheel_radius = self.wheel_diameter / 2.0
        self.wheel_circumference = math.pi * self.wheel_diameter
        
        # Initialize motor drivers
        self.get_logger().info(f"Initializing L2DB motor drivers on {self.can_channel}")
        try:
            self.motor_left = L2DBMotorDriver(
                self.can_interface, 
                self.can_channel, 
                self.motor_left_id
            )
            self.motor_right = L2DBMotorDriver(
                self.can_interface, 
                self.can_channel, 
                self.motor_right_id
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize motor drivers: {e}")
            raise
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_time = self.get_clock().now()
        self.motors_enabled = False
        self.emergency_stop = False
        
        # Current velocities
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        
        # Encoder tracking
        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.first_reading = True
        
        # Joint states for wheels
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
        # Conversion factors
        self.counts_per_revolution = self.encoder_resolution
        self.meters_per_count = self.wheel_circumference / self.counts_per_revolution
        
        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry, 
            'odom', 
            10
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # For diff_drive_controller compatibility
        self.wheel_cmd_pub = self.create_publisher(
            Float64MultiArray,
            'wheel_commands',
            10
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timers
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.update_callback
        )
        
        self.control_timer = self.create_timer(
            0.05,  # 20Hz control loop
            self.control_loop
        )
        
        # Initialize motors in a separate thread to avoid blocking
        self.init_thread = threading.Thread(target=self.initialize_motors)
        self.init_thread.start()
        
        self.get_logger().info("Wheelchair Core Node initialized")
    
    def initialize_motors(self):
        """Initialize and enable motors"""
        try:
            self.get_logger().info("Initializing motors...")
            
            # Initialize both motors
            if not self.motor_left.initialize():
                self.get_logger().error("Failed to initialize left motor")
                return
            
            time.sleep(0.2)
            
            if not self.motor_right.initialize():
                self.get_logger().error("Failed to initialize right motor")
                return
            
            # Set acceleration/deceleration
            self.motor_left.set_acceleration(self.acceleration)
            self.motor_right.set_acceleration(self.acceleration)
            self.motor_left.set_deceleration(self.deceleration)
            self.motor_right.set_deceleration(self.deceleration)
            
            # Enable motors
            time.sleep(0.5)
            
            if not self.motor_left.enable():
                self.get_logger().error("Failed to enable left motor")
                return
            
            time.sleep(0.1)
            
            if not self.motor_right.enable():
                self.get_logger().error("Failed to enable right motor")
                return
            
            self.motors_enabled = True
            self.get_logger().info("Motors initialized and enabled successfully")
            
            # Reset encoder tracking
            self.first_reading = True
            
        except Exception as e:
            self.get_logger().error(f"Motor initialization failed: {e}")
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        if self.emergency_stop:
            self.get_logger().warn("Emergency stop active, ignoring cmd_vel")
            return
        
        # Store target velocities
        self.target_linear_vel = max(-self.max_linear_vel, 
                                     min(self.max_linear_vel, msg.linear.x))
        self.target_angular_vel = max(-self.max_angular_vel, 
                                      min(self.max_angular_vel, msg.angular.z))
    
    def control_loop(self):
        """Main control loop - send velocities to motors"""
        if not self.motors_enabled:
            return
        
        if self.emergency_stop:
            self.send_wheel_velocities(0.0, 0.0)
            return
        
        # Calculate wheel velocities using differential drive kinematics
        # v_left = v_x - (omega * L) / 2
        # v_right = v_x + (omega * L) / 2
        v_left = self.target_linear_vel - (self.target_angular_vel * self.wheel_base) / 2.0
        v_right = self.target_linear_vel + (self.target_angular_vel * self.wheel_base) / 2.0
        
        # Send to motors
        self.send_wheel_velocities(v_left, v_right)
    
    def send_wheel_velocities(self, v_left_ms, v_right_ms):
        """
        Send wheel velocities to motors
        
        Args:
            v_left_ms: Left wheel velocity in m/s
            v_right_ms: Right wheel velocity in m/s
        """
        # Convert m/s to RPM
        rpm_left = (v_left_ms / self.wheel_circumference) * 60.0
        rpm_right = (v_right_ms / self.wheel_circumference) * 60.0
        
        # Send to motors (left motor negative for forward)
        try:
            self.motor_left.set_target_velocity_rpm(-rpm_left)
            self.motor_right.set_target_velocity_rpm(rpm_right)
            
            # Publish wheel commands for monitoring
            wheel_cmd = Float64MultiArray()
            wheel_cmd.data = [rpm_left, rpm_right]
            self.wheel_cmd_pub.publish(wheel_cmd)
            
        except Exception as e:
            self.get_logger().error(f"Failed to send velocities: {e}")
    
    def update_callback(self):
        """Update odometry and publish data"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Get encoder positions from motors
        if self.motors_enabled:
            try:
                # Read actual encoder positions
                left_encoder = self.motor_left.get_actual_position()
                right_encoder = self.motor_right.get_actual_position()
                
                if left_encoder is None or right_encoder is None:
                    self.get_logger().error("Failed to read encoder positions", throttle_duration_sec=1.0)
                    return
                
                # Initialize encoder values on first reading
                if self.first_reading:
                    self.last_left_encoder = left_encoder
                    self.last_right_encoder = right_encoder
                    self.first_reading = False
                    self.last_time = current_time
                    return
                
                # Calculate encoder deltas
                delta_left_counts = left_encoder - self.last_left_encoder
                delta_right_counts = right_encoder - self.last_right_encoder
                
                # Handle encoder overflow 
                MAX_ENCODER = 2147483647
                MIN_ENCODER = -2147483648
                
                if abs(delta_left_counts) > MAX_ENCODER:
                    if delta_left_counts > 0:
                        delta_left_counts = delta_left_counts - (MAX_ENCODER - MIN_ENCODER + 1)
                    else:
                        delta_left_counts = delta_left_counts + (MAX_ENCODER - MIN_ENCODER + 1)
                
                if abs(delta_right_counts) > MAX_ENCODER:
                    if delta_right_counts > 0:
                        delta_right_counts = delta_right_counts - (MAX_ENCODER - MIN_ENCODER + 1)
                    else:
                        delta_right_counts = delta_right_counts + (MAX_ENCODER - MIN_ENCODER + 1)
                
                # Convert encoder counts to distance traveled
                # Note: left motor is inverted in hardware, so negate its delta
                delta_left_distance = -delta_left_counts * self.meters_per_count
                delta_right_distance = delta_right_counts * self.meters_per_count
                
                # Calculate robot motion using differential drive kinematics
                delta_distance = (delta_left_distance + delta_right_distance) / 2.0
                delta_theta = (delta_right_distance - delta_left_distance) / self.wheel_base
                
                # Update robot pose
                if abs(delta_theta) < 1e-6:
                    # Straight line motion
                    delta_x = delta_distance * math.cos(self.theta)
                    delta_y = delta_distance * math.sin(self.theta)
                else:
                    # Arc motion
                    radius = delta_distance / delta_theta
                    delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
                    delta_y = radius * (-math.cos(self.theta + delta_theta) + math.cos(self.theta))
                
                self.x += delta_x
                self.y += delta_y
                self.theta += delta_theta
                
                # Normalize theta to [-pi, pi]
                self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
                
                # Calculate velocities for twist message
                self.vx = delta_distance / dt if dt > 0 else 0.0
                self.vth = delta_theta / dt if dt > 0 else 0.0
                
                # Calculate wheel velocities for joint states
                self.left_wheel_vel = delta_left_distance / dt if dt > 0 else 0.0
                self.right_wheel_vel = delta_right_distance / dt if dt > 0 else 0.0
                
                # Update wheel positions for joint states (in radians)
                self.left_wheel_pos += delta_left_distance / self.wheel_radius
                self.right_wheel_pos += delta_right_distance / self.wheel_radius
                
                # Store current encoder values for next iteration
                self.last_left_encoder = left_encoder
                self.last_right_encoder = right_encoder
                
                # Optional: Log encoder data for debugging
                if abs(self.vx) > 0.01 or abs(self.vth) > 0.01:
                    self.get_logger().debug(
                        f"Encoders: L={left_encoder}, R={right_encoder}, "
                        f"Deltas: L={delta_left_counts}, R={delta_right_counts}, "
                        f"Vel: {self.vx:.3f}m/s, {math.degrees(self.vth):.1f}°/s",
                        throttle_duration_sec=0.5
                    )
                
            except Exception as e:
                self.get_logger().error(f"Failed to read encoders: {e}", throttle_duration_sec=1.0)
                return
        else:
            # Motors not enabled, set velocities to zero
            self.vx = 0.0
            self.vth = 0.0
            self.left_wheel_vel = 0.0
            self.right_wheel_vel = 0.0
        
        # Publish odometry
        self.publish_odometry(current_time)
        
        # Publish joint states
        self.publish_joint_states(current_time)
        
        # Publish TF
        if self.publish_tf:
            self.publish_transform(current_time)
        
        self.last_time = current_time
    
    def publish_odometry(self, current_time):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth
        
        # Pose covariance (6x6 matrix stored as 36-element array)
        # Order: (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = self.pose_covariance_diagonal[0]   # x
        odom.pose.covariance[7] = self.pose_covariance_diagonal[1]   # y
        odom.pose.covariance[14] = self.pose_covariance_diagonal[2]  # z
        odom.pose.covariance[21] = self.pose_covariance_diagonal[3]  # roll
        odom.pose.covariance[28] = self.pose_covariance_diagonal[4]  # pitch
        odom.pose.covariance[35] = self.pose_covariance_diagonal[5]  # yaw
        
        # Twist covariance
        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = self.twist_covariance_diagonal[0]   # linear x
        odom.twist.covariance[7] = self.twist_covariance_diagonal[1]   # linear y
        odom.twist.covariance[14] = self.twist_covariance_diagonal[2]  # linear z
        odom.twist.covariance[21] = self.twist_covariance_diagonal[3]  # angular x
        odom.twist.covariance[28] = self.twist_covariance_diagonal[4]  # angular y
        odom.twist.covariance[35] = self.twist_covariance_diagonal[5]  # angular z
        
        self.odom_pub.publish(odom)
    
    def publish_joint_states(self, current_time):
        """Publish joint states for visualization"""
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        
        # Wheel joints (matching URDF names)
        joint_state.name = ['Revolute 5', 'Revolute 4']  # Left, Right from URDF
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = [self.left_wheel_vel / self.wheel_radius, 
                               self.right_wheel_vel / self.wheel_radius]
        
        self.joint_state_pub.publish(joint_state)
    
    def publish_transform(self, current_time):
        """Publish TF transform"""
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def emergency_stop_callback(self):
        """Handle emergency stop"""
        self.emergency_stop = True
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.send_wheel_velocities(0.0, 0.0)
        self.get_logger().warn("EMERGENCY STOP ACTIVATED")
    
    def release_emergency_stop(self):
        """Release emergency stop"""
        self.emergency_stop = False
        self.get_logger().info("Emergency stop released")
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down wheelchair core node...")
        
        # Stop motors
        if self.motors_enabled:
            self.send_wheel_velocities(0.0, 0.0)
            time.sleep(0.5)
            self.motor_left.disable()
            self.motor_right.disable()
        
        # Close CAN bus
        L2DBMotorDriver.close_bus()
        self.get_logger().info("Shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WheelchairCoreNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()