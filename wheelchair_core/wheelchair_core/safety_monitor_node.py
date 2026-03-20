#!/usr/bin/env python3
"""
Safety Monitor for Autonomous Wheelchair
Monitors obstacles and applies safety constraints to velocity commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Parameters
        self.declare_parameter('emergency_stop_distance', 0.3)  # meters
        self.declare_parameter('warning_distance', 0.8)  # meters
        self.declare_parameter('max_safe_speed', 0.4)  # m/s
        self.declare_parameter('check_angle_range', 120.0)  # degrees (±60°)
        self.declare_parameter('safety_timeout', 1.0)  # seconds
        
        self.emergency_distance = self.get_parameter('emergency_stop_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.max_safe_speed = self.get_parameter('max_safe_speed').value
        self.check_angle_range = math.radians(self.get_parameter('check_angle_range').value)
        self.safety_timeout = self.get_parameter('safety_timeout').value
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(
            PointCloud2, '/scan_merged', self.scan_callback, 10)
        self.joystick_emergency_sub = self.create_subscription(
            Bool, '/joystick_emergency', self.joystick_emergency_callback, 10)
        
        # Publishers
        self.safe_cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.safety_status_pub = self.create_publisher(String, '/safety_status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop_active', 10)
        
        # State
        self.last_scan_time = None
        self.emergency_stop_active = False
        self.joystick_emergency = False
        self.min_obstacle_distance = float('inf')
        self.current_cmd_vel = Twist()
        
        # Timer for safety checks
        self.safety_timer = self.create_timer(0.1, self.safety_check)  # 10Hz
        
        self.get_logger().info('Safety Monitor initialized')
        self.get_logger().info(f'Emergency stop distance: {self.emergency_distance}m')
        self.get_logger().info(f'Warning distance: {self.warning_distance}m')
    
    def cmd_vel_callback(self, msg):
        """Store incoming velocity command"""
        self.current_cmd_vel = msg
    
    def scan_callback(self, msg):
        """Process point cloud for obstacle detection"""
        self.last_scan_time = self.get_clock().now()
        
        try:
            # Convert to numpy array
            points_list = []
            for point in pc2.read_points(msg, skip_nans=True):
                x, y, z = point[0], point[1], point[2]
                
                # Check if point is in front of robot (within angle range)
                angle = math.atan2(y, x)
                if abs(angle) <= self.check_angle_range / 2:
                    # Only consider points at wheelchair height
                    if 0.1 <= z <= 1.5:  # Typical wheelchair height range
                        distance = math.sqrt(x*x + y*y)
                        points_list.append(distance)
            
            if points_list:
                self.min_obstacle_distance = min(points_list)
            else:
                self.min_obstacle_distance = float('inf')
                
        except Exception as e:
            self.get_logger().error(f'Error processing scan: {e}', throttle_duration_sec=1.0)
            self.min_obstacle_distance = 0.0  # Conservative approach
    
    def joystick_emergency_callback(self, msg):
        """Handle emergency stop from joystick"""
        self.joystick_emergency = msg.data
        if self.joystick_emergency:
            self.get_logger().warn('Emergency stop activated from joystick')
    
    def safety_check(self):
        """Main safety monitoring loop"""
        current_time = self.get_clock().now()
        
        # Check if scan data is fresh
        scan_data_fresh = (self.last_scan_time is not None and 
                          (current_time - self.last_scan_time).nanoseconds / 1e9 < self.safety_timeout)
        
        # Determine safety status
        emergency_stop_needed = False
        safety_message = "OK"
        
        # Check joystick emergency
        if self.joystick_emergency:
            emergency_stop_needed = True
            safety_message = "EMERGENCY: Joystick emergency stop active"
        
        # Check scan data freshness
        elif not scan_data_fresh:
            emergency_stop_needed = True
            safety_message = "EMERGENCY: No recent sensor data"
        
        # Check obstacle distance
        elif self.min_obstacle_distance <= self.emergency_distance:
            emergency_stop_needed = True
            safety_message = f"EMERGENCY: Obstacle at {self.min_obstacle_distance:.2f}m"
        
        # Warning zone
        elif self.min_obstacle_distance <= self.warning_distance:
            safety_message = f"WARNING: Obstacle at {self.min_obstacle_distance:.2f}m"
        
        # Apply safety constraints
        safe_cmd_vel = self.apply_safety_constraints(
            self.current_cmd_vel, emergency_stop_needed)
        
        # Publish
        self.safe_cmd_vel_pub.publish(safe_cmd_vel)
        self.emergency_stop_pub.publish(Bool(data=emergency_stop_needed))
        self.safety_status_pub.publish(String(data=safety_message))
        
        # Update state
        if emergency_stop_needed != self.emergency_stop_active:
            self.emergency_stop_active = emergency_stop_needed
            if emergency_stop_needed:
                self.get_logger().error(safety_message)
            else:
                self.get_logger().info('Emergency stop cleared')
    
    def apply_safety_constraints(self, cmd_vel, emergency_stop):
        """Apply safety constraints to velocity command"""
        safe_vel = Twist()
        
        if emergency_stop:
            # Complete stop
            return safe_vel
        
        # Copy original command
        safe_vel.linear.x = cmd_vel.linear.x
        safe_vel.linear.y = cmd_vel.linear.y
        safe_vel.linear.z = cmd_vel.linear.z
        safe_vel.angular.x = cmd_vel.angular.x
        safe_vel.angular.y = cmd_vel.angular.y
        safe_vel.angular.z = cmd_vel.angular.z
        
        # Apply speed limits based on obstacle distance
        if self.min_obstacle_distance <= self.warning_distance:
            # Scale down speed based on distance
            scale_factor = max(0.1, (self.min_obstacle_distance - self.emergency_distance) / 
                              (self.warning_distance - self.emergency_distance))
            
            # Only slow down forward motion
            if safe_vel.linear.x > 0:
                safe_vel.linear.x *= scale_factor
                safe_vel.linear.x = min(safe_vel.linear.x, self.max_safe_speed)
        
        # Apply absolute speed limits
        safe_vel.linear.x = max(-self.max_safe_speed, 
                               min(self.max_safe_speed, safe_vel.linear.x))
        
        return safe_vel


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()