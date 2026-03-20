#!/usr/bin/env python3
"""
Simple Odometry Test Script
Manual measurements for quick calibration
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import math
import time


class SimpleOdomTest(Node):
    def __init__(self):
        super().__init__('simple_odom_test')
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        
        self.reset_odom_client = self.create_client(Trigger, 'reset_odometry')
        
        self.current_odom = None
        self.start_odom = None
        
    def odom_callback(self, msg):
        self.current_odom = msg
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def reset_odometry(self):
        """Reset odometry to zero"""
        if not self.reset_odom_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Reset service not available")
            return False
        
        request = Trigger.Request()
        future = self.reset_odom_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.get_logger().info("✓ Odometry reset")
            time.sleep(0.5)
            return True
        return False
    
    def measure_straight_line(self):
        """Test straight line movement"""
        print("\n" + "="*50)
        print("STRAIGHT LINE TEST")
        print("="*50)
        print("Instructions:")
        print("1. Mark the robot's current position")
        print("2. The robot will move forward 1 meter")
        print("3. Measure the actual distance traveled")
        print("4. Measure any lateral deviation")
        
        input("\nPress Enter when ready...")
        
        # Reset odometry
        self.reset_odometry()
        
        # Move forward 1 meter at 0.2 m/s (5 seconds)
        print("Moving forward 1 meter...")
        twist = Twist()
        twist.linear.x = 0.2
        
        start_time = time.time()
        while time.time() - start_time < 5.0:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        
        # Stop
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(1.0)
        
        # Get odometry reading
        if self.current_odom:
            x = self.current_odom.pose.pose.position.x
            y = self.current_odom.pose.pose.position.y
            theta = self.quaternion_to_yaw(self.current_odom.pose.pose.orientation)
            
            print(f"\nOdometry reading:")
            print(f"  X: {x:.3f} m")
            print(f"  Y: {y:.3f} m")
            print(f"  Heading: {math.degrees(theta):.1f}°")
            print(f"  Distance: {math.sqrt(x**2 + y**2):.3f} m")
            
            # Get manual measurements
            actual_distance = float(input("\nEnter actual distance traveled (m): "))
            lateral_deviation = float(input("Enter lateral deviation (m, + for right): "))
            
            # Calculate errors
            distance_error = math.sqrt(x**2 + y**2) - actual_distance
            lateral_error = y - lateral_deviation
            
            print(f"\nErrors:")
            print(f"  Distance error: {distance_error:.3f} m ({abs(distance_error/actual_distance*100):.1f}%)")
            print(f"  Lateral error: {lateral_error:.3f} m")
            print(f"  Heading error: {math.degrees(theta):.1f}°")
            
            # Calculate scaling factor
            scale_factor = actual_distance / math.sqrt(x**2 + y**2)
            print(f"\nWheel diameter scaling factor: {scale_factor:.4f}")
            print(f"  If consistent, multiply wheel_diameter by this factor")
            
            return {
                'odom_x': x,
                'odom_y': y,
                'odom_distance': math.sqrt(x**2 + y**2),
                'actual_distance': actual_distance,
                'lateral_deviation': lateral_deviation,
                'distance_error': distance_error,
                'lateral_error': lateral_error,
                'scale_factor': scale_factor
            }
    
    def measure_rotation(self):
        """Test rotation"""
        print("\n" + "="*50)
        print("ROTATION TEST")
        print("="*50)
        print("Instructions:")
        print("1. Mark the robot's current heading")
        print("2. The robot will rotate 360 degrees")
        print("3. Check if it returns to the same heading")
        
        input("\nPress Enter when ready...")
        
        # Reset odometry
        self.reset_odometry()
        
        # Rotate 360 degrees at 0.349066 rad/s (~21 seconds)
        print("Rotating 360 degrees...")
        twist = Twist()
        twist.angular.z = 0.349066
        
        duration = (2 * math.pi) / 0.349066
        start_time = time.time()
        
        x = self.current_odom.pose.pose.position.x
        y = self.current_odom.pose.pose.position.y
        theta_prev = self.quaternion_to_yaw(self.current_odom.pose.pose.orientation)

        angle_accumulated = 0.0

        while angle_accumulated < 2 * math.pi:  # full rotation in radians
            
            self.cmd_vel_pub.publish(twist)

            theta_curr = self.quaternion_to_yaw(self.current_odom.pose.pose.orientation)

            # Compute change in angle (handle wrap-around from -pi to pi)
            delta_theta = theta_curr - theta_prev
            if delta_theta < -math.pi:
                delta_theta += 2 * math.pi
            elif delta_theta > math.pi:
                delta_theta -= 2 * math.pi

            angle_accumulated += abs(delta_theta)
            theta_prev = theta_curr

            time.sleep(0.05)

        # Stop
        twist.angular.z = 0.0
        finish_time = time.time()
        print(f"Rotation completed in {finish_time - start_time:.2f} seconds")
        self.cmd_vel_pub.publish(twist)
        time.sleep(1.0)

        
        # Get odometry reading
        if self.current_odom:
            x = self.current_odom.pose.pose.position.x
            y = self.current_odom.pose.pose.position.y
            theta = self.quaternion_to_yaw(self.current_odom.pose.pose.orientation)
            
            print(f"\nOdometry reading:")
            print(f"  Final angle: {math.degrees(theta):.1f}°")
            print(f"  Position drift: ({x:.3f}, {y:.3f}) m")
            print(f"  Distance from origin: {math.sqrt(x**2 + y**2):.3f} m")
            
            # Get manual measurement
            actual_rotation = float(input("\nEnter actual rotation (degrees, + for CCW): "))
            
            # Calculate error
            angle_error = math.degrees(theta) - actual_rotation
            
            print(f"\nErrors:")
            print(f"  Angle error: {angle_error:.1f}°")
            print(f"  Position drift: {math.sqrt(x**2 + y**2):.3f} m")
            
            # Calculate scaling factor
            scale_factor = actual_rotation / math.degrees(theta) if theta != 0 else 1.0
            print(f"\nWheel base scaling factor: {scale_factor:.4f}")
            print(f"  If consistent, multiply wheel_base by this factor")
            
            return {
                'odom_angle': math.degrees(theta),
                'actual_angle': actual_rotation,
                'angle_error': angle_error,
                'position_drift': math.sqrt(x**2 + y**2),
                'scale_factor': scale_factor
            }
    
    def calculate_covariance(self, results_list):
        """Calculate covariance from multiple measurements"""
        if not results_list:
            return None
        
        print("\n" + "="*50)
        print("COVARIANCE CALCULATION")
        print("="*50)
        
        # Extract errors
        distance_errors = [r.get('distance_error', 0) for r in results_list if 'distance_error' in r]
        lateral_errors = [r.get('lateral_error', 0) for r in results_list if 'lateral_error' in r]
        angle_errors = [r.get('angle_error', 0) for r in results_list if 'angle_error' in r]
        
        # Calculate variances
        import numpy as np
        
        if distance_errors:
            var_x = np.var(distance_errors)
            std_x = np.std(distance_errors)
            print(f"Distance variance: {var_x:.6f} (std: {std_x:.3f})")
        else:
            var_x = 0.001
        
        if lateral_errors:
            var_y = np.var(lateral_errors)
            std_y = np.std(lateral_errors)
            print(f"Lateral variance: {var_y:.6f} (std: {std_y:.3f})")
        else:
            var_y = 0.001
        
        if angle_errors:
            var_theta = np.var([math.radians(a) for a in angle_errors])
            std_theta = np.std([math.radians(a) for a in angle_errors])
            print(f"Angle variance: {var_theta:.6f} (std: {math.degrees(std_theta):.1f}°)")
        else:
            var_theta = 0.001
        
        # Create covariance values (using 3-sigma for 99.7% confidence)
        pose_cov_diagonal = [
            (3 * std_x)**2 if distance_errors else 0.01,  # x
            (3 * std_y)**2 if lateral_errors else 0.01,   # y
            0.001,  # z
            0.001,  # roll
            0.001,  # pitch
            (3 * std_theta)**2 if angle_errors else 0.01  # yaw
        ]
        
        twist_cov_diagonal = [
            0.001,  # linear x
            0.001,  # linear y
            0.001,  # linear z
            0.001,  # angular x
            0.001,  # angular y
            0.001   # angular z
        ]
        
        print("\nRecommended covariance diagonal values:")
        print("Pose covariance:")
        for i, val in enumerate(pose_cov_diagonal):
            labels = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
            print(f"  {labels[i]}: {val:.6f}")
        
        print("\nAdd to wheelchair_params.yaml:")
        print("  pose_covariance_diagonal:", pose_cov_diagonal)
        print("  twist_covariance_diagonal:", twist_cov_diagonal)
        
        return pose_cov_diagonal, twist_cov_diagonal
    
    def run_manual_calibration(self):
        """Run manual calibration tests"""
        print("\n" + "="*60)
        print("MANUAL ODOMETRY CALIBRATION")
        print("="*60)
        print("\nYou will need:")
        print("- Tape measure")
        print("- Masking tape to mark positions")
        print("- Clear floor space (at least 2m x 2m)")
        
        results = []
        
        while True:
            print("\n" + "="*40)
            print("Select test:")
            print("1. Straight line test")
            print("2. Rotation test")
            print("3. Calculate covariances")
            print("4. Exit")
            
            choice = input("\nEnter choice (1-4): ")
            
            if choice == '1':
                result = self.measure_straight_line()
                if result:
                    results.append(result)
                    print(f"\nTest {len(results)} completed")
            
            elif choice == '2':
                result = self.measure_rotation()
                if result:
                    results.append(result)
                    print(f"\nTest {len(results)} completed")
            
            elif choice == '3':
                if results:
                    self.calculate_covariance(results)
                else:
                    print("No test results available. Run some tests first!")
            
            elif choice == '4':
                break
            
            else:
                print("Invalid choice")
        
        print("\nCalibration complete!")
        return results


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleOdomTest()
    
    try:
        # Run in a thread to keep ROS spinning
        import threading
        
        def test_thread():
            time.sleep(1)
            node.run_manual_calibration()
            print("\nPress Ctrl+C to exit")
        
        thread = threading.Thread(target=test_thread)
        thread.start()
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()