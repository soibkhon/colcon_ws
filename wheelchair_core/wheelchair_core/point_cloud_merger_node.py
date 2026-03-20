#!/usr/bin/env python3
"""
Point Cloud Merger for Dual Livox 360 Lidars
Merges /livox/lidar_192_168_1_105 and /livox/lidar_192_168_1_172
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
from builtin_interfaces.msg import Time
import struct


class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('point_cloud_merger')
        
        # Parameters
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('max_range', 20.0)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('merge_timeout', 0.1)  # seconds
        
        self.output_frame = self.get_parameter('output_frame').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.merge_timeout = self.get_parameter('merge_timeout').value
        
        # Subscribers for your specific Livox lidars
        self.left_sub = self.create_subscription(
            PointCloud2, 
            '/livox/lidar_192_168_1_172',  # Left lidar
            self.left_callback, 
            10
        )
        self.right_sub = self.create_subscription(
            PointCloud2, 
            '/livox/lidar_192_168_1_105',  # Right lidar
            self.right_callback, 
            10
        )
        
        # Publishers
        self.merged_pub = self.create_publisher(PointCloud2, '/scan_merged', 10)
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Data storage
        self.left_cloud = None
        self.right_cloud = None
        self.left_timestamp = None
        self.right_timestamp = None
        
        # Timer for periodic merging
        self.merge_timer = self.create_timer(0.05, self.merge_clouds)  # 20Hz
        
        self.get_logger().info('Point Cloud Merger initialized')
        self.get_logger().info(f'Subscribing to:')
        self.get_logger().info(f'  - /livox/lidar_192_168_1_105 (right)')
        self.get_logger().info(f'  - /livox/lidar_192_168_1_172 (left)')
        self.get_logger().info(f'Publishing to: /scan_merged')
    
    def left_callback(self, msg):
        """Store left lidar data"""
        self.left_cloud = msg
        self.left_timestamp = self.get_clock().now()
    
    def right_callback(self, msg):
        """Store right lidar data"""
        self.right_cloud = msg
        self.right_timestamp = self.get_clock().now()
    
    def merge_clouds(self):
        """Merge point clouds from both lidars"""
        current_time = self.get_clock().now()
        
        # Check if we have recent data from both lidars
        if self.left_cloud is None or self.right_cloud is None:
            return
        
        # Check data freshness
        if (current_time - self.left_timestamp).nanoseconds / 1e9 > self.merge_timeout:
            self.get_logger().warn('Left lidar data too old', throttle_duration_sec=2.0)
            return
        
        if (current_time - self.right_timestamp).nanoseconds / 1e9 > self.merge_timeout:
            self.get_logger().warn('Right lidar data too old', throttle_duration_sec=2.0)
            return
        
        try:
            # Convert point clouds to numpy arrays
            left_points = self.pointcloud_to_numpy(self.left_cloud)
            right_points = self.pointcloud_to_numpy(self.right_cloud)
            
            if left_points is None or right_points is None:
                return
            
            # Transform to base_link frame
            left_transformed = self.transform_points(left_points, 
                                                   self.left_cloud.header.frame_id,
                                                   self.output_frame)
            right_transformed = self.transform_points(right_points,
                                                    self.right_cloud.header.frame_id, 
                                                    self.output_frame)
            
            if left_transformed is None or right_transformed is None:
                return
            
            # Merge point clouds
            merged_points = np.vstack([left_transformed, right_transformed])
            
            # Filter by range
            distances = np.linalg.norm(merged_points[:, :3], axis=1)
            valid_mask = (distances >= self.min_range) & (distances <= self.max_range)
            filtered_points = merged_points[valid_mask]
            
            # Create merged point cloud message
            merged_msg = self.numpy_to_pointcloud(filtered_points, self.output_frame)
            merged_msg.header.stamp = current_time.to_msg()
            
            # Publish
            self.merged_pub.publish(merged_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error merging clouds: {e}', throttle_duration_sec=1.0)
    
    def pointcloud_to_numpy(self, cloud_msg):
        """Convert PointCloud2 to numpy array"""
        try:
            # Extract points
            points_list = []
            for point in pc2.read_points(cloud_msg, skip_nans=True):
                points_list.append([point[0], point[1], point[2]])
            
            if len(points_list) == 0:
                return None
                
            return np.array(points_list, dtype=np.float32)
            
        except Exception as e:
            self.get_logger().error(f'Error converting pointcloud: {e}')
            return None
    
    def transform_points(self, points, source_frame, target_frame):
        """Transform points from source frame to target frame"""
        try:
            # Get transform
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            
            # Extract translation and rotation
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            # Convert quaternion to rotation matrix
            rotation_matrix = self.quaternion_to_rotation_matrix(
                rot.x, rot.y, rot.z, rot.w)
            
            # Apply transformation
            transformed_points = np.dot(points, rotation_matrix.T)
            transformed_points[:, 0] += trans.x
            transformed_points[:, 1] += trans.y
            transformed_points[:, 2] += trans.z
            
            return transformed_points
            
        except Exception as e:
            self.get_logger().error(f'Transform error {source_frame}->{target_frame}: {e}',
                                  throttle_duration_sec=2.0)
            return None
    
    def quaternion_to_rotation_matrix(self, x, y, z, w):
        """Convert quaternion to rotation matrix"""
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
    
    def numpy_to_pointcloud(self, points, frame_id):
        """Convert numpy array to PointCloud2"""
        header = self.left_cloud.header  # Use left cloud as template
        header.frame_id = frame_id
        
        # Define fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Create point cloud
        cloud_msg = pc2.create_cloud(header, fields, points)
        return cloud_msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()