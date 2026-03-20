#!/usr/bin/env python3
"""
Position Controller Node for Wheelchair
Sends velocity commands to achieve target positions
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import math
import time


class PositionControllerNode(Node):
    """Position controller for wheelchair - moves to specific distances/angles"""
    
    def __init__(self):
        super().__init__('position_controller')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_tolerance', 0.001),      # 1cm position tolerance
                ('angular_tolerance', 0.017),     # ~1 degree angle tolerance
                ('max_linear_velocity', 0.3),    # m/s for position control
                ('max_angular_velocity', 0.3),   # rad/s for position control
                ('linear_kp', 0.5),              # Proportional gain for linear
                ('angular_kp', 2.0),             # Proportional gain for angular
                ('timeout', 25.0),               # Timeout for reaching target
            ]
        )
        
        # Get parameters
        self.linear_tolerance = self.get_parameter('linear_tolerance').value
        self.angular_tolerance = self.get_parameter('angular_tolerance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.timeout = self.get_parameter('timeout').value
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = None
        self.is_moving = False
        self.start_time = None
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'position_status', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Subscriber for goal poses
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'move_to_pose',
            self.goal_pose_callback,
            10
        )
        
        # Services for simple movements
        self.create_service(Trigger, 'move_forward_2m', self.move_forward_2m)
        self.create_service(Trigger, 'move_backward_2m', self.move_backward_2m)
        self.create_service(Trigger, 'turn_left_90', self.turn_left_90)
        self.create_service(Trigger, 'turn_right_90', self.turn_right_90)
        self.create_service(Trigger, 'stop_movement', self.stop_movement)
        
        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        self.get_logger().info('Position Controller initialized')
        self.print_usage()
    
    def print_usage(self):
        """Print usage instructions"""
        self.get_logger().info('='*60)
        self.get_logger().info('POSITION CONTROLLER USAGE')
        self.get_logger().info('='*60)
        self.get_logger().info('Services:')
        self.get_logger().info('  ros2 service call /move_forward_2m std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /move_backward_2m std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /turn_left_90 std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /turn_right_90 std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /stop_movement std_srvs/srv/Trigger')
        self.get_logger().info('')
        self.get_logger().info('Topic (for custom positions):')
        self.get_logger().info('  ros2 topic pub /move_to_pose geometry_msgs/msg/PoseStamped ...')
        self.get_logger().info('='*60)
    
    def odom_callback(self, msg):
        """Update current position from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
    
    def goal_pose_callback(self, msg):
        """Handle goal pose request"""
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        
        # Extract target orientation if provided
        q = msg.pose.orientation
        if q.w != 0 or q.x != 0 or q.y != 0 or q.z != 0:
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.target_theta = math.atan2(siny_cosp, cosy_cosp)
        else:
            self.target_theta = None
        
        self.is_moving = True
        self.start_time = time.time()
        
        self.get_logger().info(f'Moving to position: ({self.target_x:.2f}, {self.target_y:.2f})')

    def move_forward_2m(self, request, response):
        """Service callback to move forward 2 meters"""
        # Calculate target position 2m forward in current direction
        self.target_x = self.current_x + 2.0 * math.cos(self.current_theta)
        self.target_y = self.current_y + 2.0 * math.sin(self.current_theta)
        self.target_theta = None  # Don't change orientation
        
        self.is_moving = True
        self.start_time = time.time()
        
        response.success = True
        response.message = 'Moving forward 2 meters'
        self.get_logger().info('Starting forward movement: 2 meters')
        return response

    def move_backward_2m(self, request, response):
        """Service callback to move backward 2 meters"""
        # Calculate target position 2m backward in current direction
        self.target_x = self.current_x - 2.0 * math.cos(self.current_theta)
        self.target_y = self.current_y + 2.0 * math.sin(self.current_theta)
        self.target_theta = None  # Don't change orientation
        
        self.is_moving = True
        self.start_time = time.time()
        
        response.success = True
        response.message = 'Moving backward 2 meters'
        self.get_logger().info('Starting backward movement: 2 meters')
        return response
    
    def turn_left_90(self, request, response):
        """Service callback to turn left 90 degrees"""
        self.target_x = self.current_x  # Stay in place
        self.target_y = self.current_y  # Stay in place
        self.target_theta = self.current_theta + math.pi/2  # Turn 90 degrees left
        
        self.is_moving = True
        self.start_time = time.time()
        
        response.success = True
        response.message = 'Turning left 90 degrees'
        self.get_logger().info('Starting left turn: 90 degrees')
        return response
    
    def turn_right_90(self, request, response):
        """Service callback to turn right 90 degrees"""
        self.target_x = self.current_x  # Stay in place
        self.target_y = self.current_y  # Stay in place
        self.target_theta = self.current_theta - math.pi/2  # Turn 90 degrees right
        
        self.is_moving = True
        self.start_time = time.time()
        
        response.success = True
        response.message = 'Turning right 90 degrees'
        self.get_logger().info('Starting right turn: 90 degrees')
        return response
    
    def stop_movement(self, request, response):
        """Service callback to stop movement"""
        self.is_moving = False
        self.stop_robot()
        
        response.success = True
        response.message = 'Movement stopped'
        self.get_logger().info('Movement stopped by user')
        return response
    
    def control_loop(self):
        """Main control loop for position control"""
        if not self.is_moving:
            return
        
        # Check timeout
        if self.start_time and (time.time() - self.start_time) > self.timeout:
            self.get_logger().warn('Movement timeout - stopping')
            self.is_moving = False
            self.stop_robot()
            return
        
        # Calculate errors
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance_error = math.sqrt(dx*dx + dy*dy)
        
        # Calculate angle to target
        angle_to_target = math.atan2(dy, dx)
        
        # Determine if we need to rotate or move
        twist = Twist()
        
        # First, check if we need to reach a specific orientation
        if self.target_theta is not None:
            # Pure rotation mode (when at target position)
            if distance_error < self.linear_tolerance:
                angle_error = self.target_theta - self.current_theta
                # Normalize angle error to [-pi, pi]
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
                
                if abs(angle_error) < self.angular_tolerance:
                    # Reached target position and orientation
                    self.get_logger().info('Target reached!')
                    self.is_moving = False
                    self.stop_robot()
                    self.publish_status("Target reached")
                    return
                
                # Pure rotation to reach target orientation
                twist.angular.z = self.angular_kp * angle_error
                twist.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, twist.angular.z))
            else:
                # Move to position first
                self.move_to_position(twist, distance_error, angle_to_target)
        else:
            # Just move to position without specific final orientation
            if distance_error < self.linear_tolerance:
                # Reached target position
                self.get_logger().info('Target position reached!')
                self.is_moving = False
                self.stop_robot()
                self.publish_status("Target position reached")
                return
            
            self.move_to_position(twist, distance_error, angle_to_target)
        
        # Send velocity command
        self.cmd_vel_pub.publish(twist)
        
        # Publish status
        status_msg = f"Distance to target: {distance_error:.3f}m"
        if self.target_theta is not None:
            angle_error = self.target_theta - self.current_theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            status_msg += f", Angle error: {math.degrees(angle_error):.1f}°"
        self.publish_status(status_msg)
    
    def move_to_position(self, twist, distance_error, angle_to_target):
        """Calculate twist command to move to position"""
        # Calculate angle error for facing the target
        angle_error = angle_to_target - self.current_theta
        # Normalize angle error to [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # If angle error is large, rotate first
        if abs(angle_error) > 0.2:  # ~11 degrees
            # Rotate to face target
            twist.angular.z = self.angular_kp * angle_error
            twist.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, twist.angular.z))
            # Slow linear movement while turning
            twist.linear.x = 0.1 * self.linear_kp * distance_error
        else:
            # Move forward/backward toward target
            twist.linear.x = self.linear_kp * distance_error
            twist.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, twist.linear.x))
            # Adjust heading while moving
            twist.angular.z = self.angular_kp * angle_error
            twist.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, twist.angular.z))
    
    def stop_robot(self):
        """Send stop command"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def publish_status(self, message):
        """Publish status message"""
        status = String()
        status.data = message
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PositionControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()