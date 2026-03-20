#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import message_filters


class SimpleDualLaserMerger(Node):
    def __init__(self):
        super().__init__('simple_dual_laser_merger')
        
        # Parameters
        self.declare_parameter('output_topic', '/livox/lidar')
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        
        # Subscribers with message filters for synchronization
        self.cloud1_sub = message_filters.Subscriber(self, PointCloud2, 'input_cloud1')
        self.cloud2_sub = message_filters.Subscriber(self, PointCloud2, 'input_cloud2')
        
        # Time synchronizer
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.cloud1_sub, self.cloud2_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.sync.registerCallback(self.merge_callback)
        
        # Publisher
        self.merged_pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        
        self.get_logger().info(f'Simple Dual Laser Merger started, publishing to: {self.output_topic}')

    def merge_callback(self, cloud1_msg, cloud2_msg):
        """Simply merge two point clouds by concatenating their data"""
        try:
            # Create merged message based on first cloud
            merged_msg = PointCloud2()
            merged_msg.header = cloud1_msg.header
            merged_msg.fields = cloud1_msg.fields
            merged_msg.height = 1
            merged_msg.width = cloud1_msg.width + cloud2_msg.width
            merged_msg.point_step = cloud1_msg.point_step
            merged_msg.row_step = merged_msg.point_step * merged_msg.width
            merged_msg.is_dense = cloud1_msg.is_dense and cloud2_msg.is_dense
            
            # Simply concatenate the raw data
            merged_msg.data = cloud1_msg.data + cloud2_msg.data
            
            # Publish
            self.merged_pub.publish(merged_msg)
            
            # Log occasionally
            if merged_msg.width % 1000 == 0:  # Log every 1000 points
                self.get_logger().info(f'Merged {cloud1_msg.width} + {cloud2_msg.width} = {merged_msg.width} points')
            
        except Exception as e:
            self.get_logger().error(f'Error merging clouds: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDualLaserMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()