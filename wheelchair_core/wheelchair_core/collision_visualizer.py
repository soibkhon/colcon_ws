#!/usr/bin/env python3
"""
collision_visualizer.py - Visualization node for collision avoidance system
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math
import numpy as np


class CollisionVisualizer(Node):
    
    def __init__(self):
        super().__init__('collision_visualizer')
        
        # Parameters
        self.declare_parameter('safety_distance', 0.6)
        self.declare_parameter('warning_distance', 1.2)
        self.declare_parameter('visualization_frequency', 10.0)
        self.declare_parameter('use_sim_time', False)
        
        self.safety_distance = self.get_parameter('safety_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.viz_freq = self.get_parameter('visualization_frequency').value
        
        # State
        self.last_scan = None
        self.robot_pose = None
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'collision_markers', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self._scan_callback, 10)
        self.pose_sub = self.create_subscription(Odometry, 'robot_pose', self._pose_callback, 10)
        
        # Timer
        self.viz_timer = self.create_timer(1.0 / self.viz_freq, self._publish_markers)
        
        self.get_logger().info("Collision Visualizer initialized")
    
    def _scan_callback(self, msg):
        """Handle laser scan data"""
        self.last_scan = msg
    
    def _pose_callback(self, msg):
        """Handle robot pose data"""
        self.robot_pose = msg.pose.pose
    
    def _publish_markers(self):
        """Publish visualization markers"""
        if not self.last_scan or not self.robot_pose:
            return
        
        marker_array = MarkerArray()
        
        # Clear existing markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Safety zone marker (red circle)
        safety_marker = self._create_circle_marker(
            0, self.safety_distance, 
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3),
            "Safety Zone"
        )
        marker_array.markers.append(safety_marker)
        
        # Warning zone marker (yellow circle)
        warning_marker = self._create_circle_marker(
            1, self.warning_distance,
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.2),
            "Warning Zone"
        )
        marker_array.markers.append(warning_marker)
        
        # Obstacle markers from laser scan
        obstacle_markers = self._create_obstacle_markers()
        marker_array.markers.extend(obstacle_markers)
        
        # Sector visualization
        sector_markers = self._create_sector_markers()
        marker_array.markers.extend(sector_markers)
        
        self.marker_pub.publish(marker_array)
    
    def _create_circle_marker(self, marker_id, radius, color, description):
        """Create a circular zone marker"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "collision_zones"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Scale
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = 0.1
        
        # Color
        marker.color = color
        
        return marker
    
    def _create_obstacle_markers(self):
        """Create markers for detected obstacles"""
        if not self.last_scan:
            return []
        
        markers = []
        ranges = np.array(self.last_scan.ranges)
        ranges = np.where(np.isinf(ranges), self.last_scan.range_max, ranges)
        ranges = np.where(np.isnan(ranges), self.last_scan.range_max, ranges)
        
        angle_min = self.last_scan.angle_min
        angle_increment = self.last_scan.angle_increment
        
        obstacle_id = 100
        for i, distance in enumerate(ranges):
            if distance < self.warning_distance:
                angle = angle_min + i * angle_increment
                
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "obstacles"
                marker.id = obstacle_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                # Position in robot frame
                marker.pose.position.x = distance * math.cos(angle)
                marker.pose.position.y = distance * math.sin(angle)
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0
                
                # Scale
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                
                # Color based on distance
                if distance < self.safety_distance:
                    marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red
                else:
                    marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)  # Orange
                
                markers.append(marker)
                obstacle_id += 1
        
        return markers
    
    def _create_sector_markers(self):
        """Create sector visualization markers"""
        markers = []
        
        # Define sectors
        sectors = [
            ("Front", -30, 30, ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)),
            ("Left", 30, 120, ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.3)),
            ("Right", -120, -30, ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.3)),
            ("Back", 120, 240, ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.3)),
        ]
        
        sector_id = 200
        for name, start_angle, end_angle, color in sectors:
            marker = self._create_sector_marker(
                sector_id, name, 
                math.radians(start_angle), math.radians(end_angle), 
                self.warning_distance, color
            )
            markers.append(marker)
            sector_id += 1
        
        return markers
    
    def _create_sector_marker(self, marker_id, name, start_angle, end_angle, radius, color):
        """Create a sector visualization marker"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sectors"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Create arc points
        points = []
        num_points = 20
        angle_step = (end_angle - start_angle) / num_points
        
        # Start from center
        points.append(Point(x=0.0, y=0.0, z=0.0))
        
        # Arc points
        for i in range(num_points + 1):
            angle = start_angle + i * angle_step
            point = Point()
            point.x = radius * math.cos(angle)
            point.y = radius * math.sin(angle)
            point.z = 0.0
            points.append(point)
        
        # Back to center
        points.append(Point(x=0.0, y=0.0, z=0.0))
        
        marker.points = points
        
        # Scale
        marker.scale.x = 0.02  # Line width
        
        # Color
        marker.color = color
        
        return marker


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = None
    try:
        node = CollisionVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()