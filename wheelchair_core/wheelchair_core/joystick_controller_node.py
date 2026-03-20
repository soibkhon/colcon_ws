#!/usr/bin/env python3
"""
ROS2 Joystick Controller Node for Wheelchair using pygame
Directly interfaces with Bluetooth joystick without requiring joy node
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool
import pygame
import threading
import math
import time


class JoystickControllerNode(Node):
    """ROS2 node for direct joystick control of wheelchair using pygame"""
    
    def __init__(self):
        super().__init__('joystick_controller')
        
        # Initialize pygame
        pygame.init()
        pygame.joystick.init()
        
        # Check for joysticks
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No gamepad detected! Please connect a gamepad.")
            self.get_logger().info("Available joysticks: 0")
            self.gamepad = None
        else:
            # Initialize the first joystick
            self.gamepad = pygame.joystick.Joystick(0)
            self.gamepad.init()
            self.get_logger().info(f"Gamepad detected: {self.gamepad.get_name()}")
            self.get_logger().info(f"Axes: {self.gamepad.get_numaxes()}, Buttons: {self.gamepad.get_numbuttons()}")
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('deadzone', 0.25),           # Joystick deadzone
                ('max_linear_velocity', 0.83), # m/s (3 km/h)
                ('max_angular_velocity', 0.5), # rad/s
                ('speed_increment', 0.1),      # Speed adjustment increment
                ('publish_rate', 20.0),        # Hz for pygame and cmd_vel
                ('enable_safety', True),       # Require enable button
                ('scale_linear', 0.5),         # Start at 50% speed
                ('scale_angular', 0.8),        # Start at 80% turn rate
            ]
        )
        
        # Get parameters
        self.deadzone = self.get_parameter('deadzone').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.speed_increment = self.get_parameter('speed_increment').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_safety = self.get_parameter('enable_safety').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value
        
        # Button mapping (based on common gamepad layouts)
        self.button_map = {
            'enable': 4,        # Left bumper (L1) - hold to move
            'turbo': 5,         # Right bumper (R1) - turbo mode
            'emergency': 0,     # A button (X on PS) - emergency stop
            'speed_up': 3,      # Y button (Triangle on PS) - increase speed
            'speed_down': 1,    # B button (Circle on PS) - decrease speed
            'reset_speed': 2,   # X button (Square on PS) - reset to default
        }
        
        # Axis mapping
        self.axis_map = {
            'left_x': 0,        # Left stick horizontal
            'left_y': 1,        # Left stick vertical
            'right_x': 3,       # Right stick horizontal (alternative control)
            'right_y': 4,       # Right stick vertical (alternative control)
            'left_trigger': 2,  # Left trigger (L2) - brake
            'right_trigger': 5, # Right trigger (R2) - brake
        }
        
        # State variables
        self.enabled = False
        self.turbo_mode = False
        self.emergency_stop = False
        self.last_button_press = {}
        self.current_linear_scale = self.scale_linear
        self.current_angular_scale = self.scale_angular
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            'joystick_status',
            10
        )
        
        # Services
        self.emergency_stop_srv = self.create_service(
            Trigger,
            'joystick_emergency_stop',
            self.emergency_stop_callback
        )
        
        # Start pygame event loop in separate thread
        if self.gamepad:
            self.running = True
            self.pygame_thread = threading.Thread(target=self.pygame_loop)
            self.pygame_thread.daemon = True
            self.pygame_thread.start()
            
            # Timer for publishing cmd_vel
            self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_cmd_vel)
            
            self.print_controls()
        else:
            self.get_logger().error("No gamepad found. Please connect a Bluetooth gamepad and restart.")
            self.running = False
    
    def print_controls(self):
        """Print control mapping"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('WHEELCHAIR JOYSTICK CONTROLS (Bluetooth Gamepad)')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Left Stick: Move (Forward/Back/Turn)')
        self.get_logger().info('Right Stick: Alternative control')
        self.get_logger().info('L1 (hold): Enable movement')
        self.get_logger().info('R1 (hold): Turbo mode')
        self.get_logger().info('A/X Button: Emergency stop')
        self.get_logger().info('Y/Triangle: Increase speed scale')
        self.get_logger().info('B/Circle: Decrease speed scale')
        self.get_logger().info('X/Square: Reset speed to default')
        self.get_logger().info('L2/R2: Analog braking (if supported)')
        self.get_logger().info(f'Current speed scale: Linear={self.current_linear_scale:.1f}, Angular={self.current_angular_scale:.1f}')
        self.get_logger().info(f'Max speed: {self.max_linear_vel:.2f} m/s, {self.max_angular_vel:.2f} rad/s')
        self.get_logger().info('=' * 50)
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone:
            return 0.0
        
        # Scale value to account for deadzone
        if value > 0:
            return (value - self.deadzone) / (1.0 - self.deadzone)
        else:
            return (value + self.deadzone) / (1.0 - self.deadzone)
    
    def pygame_loop(self):
        """Pygame event loop to read joystick"""
        clock = pygame.time.Clock()
        
        while self.running:
            try:
                pygame.event.pump()
                
                # Read joystick state
                if self.gamepad:
                    # Check buttons
                    current_time = time.time()
                    
                    
                    self.enabled = True #enable the joystick control
                    
                    # Turbo mode
                    self.turbo_mode = self.gamepad.get_button(self.button_map['turbo'])
                    
                    # Emergency stop (toggle)
                    if self.gamepad.get_button(self.button_map['emergency']):
                        if 'emergency' not in self.last_button_press or \
                           current_time - self.last_button_press['emergency'] > 0.5:
                            self.emergency_stop = not self.emergency_stop
                            if self.emergency_stop:
                                self.get_logger().warn('EMERGENCY STOP ACTIVATED')
                            else:
                                self.get_logger().info('Emergency stop released')
                            self.last_button_press['emergency'] = current_time
                    
                    # Speed adjustment
                    if self.gamepad.get_button(self.button_map['speed_up']):
                        if 'speed_up' not in self.last_button_press or \
                           current_time - self.last_button_press['speed_up'] > 0.3:
                            self.current_linear_scale = min(1.0, self.current_linear_scale + self.speed_increment)
                            self.current_angular_scale = min(1.0, self.current_angular_scale + self.speed_increment)
                            self.get_logger().info(f'Speed increased: Linear={self.current_linear_scale:.1f}, Angular={self.current_angular_scale:.1f}')
                            self.last_button_press['speed_up'] = current_time
                    
                    if self.gamepad.get_button(self.button_map['speed_down']):
                        if 'speed_down' not in self.last_button_press or \
                           current_time - self.last_button_press['speed_down'] > 0.3:
                            self.current_linear_scale = max(0.1, self.current_linear_scale - self.speed_increment)
                            self.current_angular_scale = max(0.1, self.current_angular_scale - self.speed_increment)
                            self.get_logger().info(f'Speed decreased: Linear={self.current_linear_scale:.1f}, Angular={self.current_angular_scale:.1f}')
                            self.last_button_press['speed_down'] = current_time
                    
                    if self.gamepad.get_button(self.button_map['reset_speed']):
                        if 'reset_speed' not in self.last_button_press or \
                           current_time - self.last_button_press['reset_speed'] > 0.5:
                            self.current_linear_scale = self.scale_linear
                            self.current_angular_scale = self.scale_angular
                            self.get_logger().info('Speed reset to default')
                            self.last_button_press['reset_speed'] = current_time
                
                clock.tick(int(self.publish_rate))
                
            except Exception as e:
                self.get_logger().error(f'Error in pygame loop: {e}')
                time.sleep(0.1)
    
    def publish_cmd_vel(self):
        """Publish velocity command based on joystick state"""
        twist = Twist()
        
        if not self.gamepad or self.emergency_stop:
            # Stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return
        
        # Only move if enabled (safety button pressed)
        if not self.enabled:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            # Publish status
            status = String()
            status.data = "Waiting - Hold L1 to enable movement"
            self.status_pub.publish(status)
            return
        
        try:
            # Get joystick values (use left stick by default)
            linear_val = -self.gamepad.get_axis(self.axis_map['left_y'])  # Negative for forward
            angular_val = -self.gamepad.get_axis(self.axis_map['left_x'])  # Negative for correct turn direction
            
            # Apply deadzone
            linear_val = self.apply_deadzone(linear_val)
            angular_val = self.apply_deadzone(angular_val)
            
            # Calculate velocities
            twist.linear.x = linear_val * self.max_linear_vel * self.current_linear_scale
            twist.angular.z = angular_val * self.max_angular_vel * self.current_angular_scale
            
            # Apply turbo mode
            if self.turbo_mode:
                twist.linear.x *= 2
                twist.angular.z *= 2
                
                # Still respect absolute max limits
                twist.linear.x = max(-self.max_linear_vel * 2,
                                     min(self.max_linear_vel * 2, twist.linear.x))
                twist.angular.z = max(-self.max_angular_vel * 2,
                                      min(self.max_angular_vel * 2, twist.angular.z))

            # Publish command
            self.cmd_vel_pub.publish(twist)
            
            # Publish status if moving
            if abs(twist.linear.x) > 0.01 or abs(twist.angular.z) > 0.01:
                status = String()
                mode = "TURBO" if self.turbo_mode else "NORMAL"
                status.data = f'{mode} - Linear: {twist.linear.x:.2f} m/s, Angular: {twist.angular.z:.2f} rad/s'
                self.status_pub.publish(status)
                
        except Exception as e:
            self.get_logger().error(f'Error reading joystick: {e}', throttle_duration_sec=1.0)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
    
    def emergency_stop_callback(self, request, response):
        """Handle emergency stop service"""
        self.emergency_stop = True
        self.get_logger().warn('Emergency stop activated via service')
        response.success = True
        response.message = 'Emergency stop activated'
        return response
    
    def release_stop_callback(self, request, response):
        """Handle release emergency stop service"""
        self.emergency_stop = False
        self.get_logger().info('Emergency stop released')
        response.success = True
        response.message = 'Emergency stop released'
        return response
    
    def shutdown(self):
        """Clean shutdown"""
        self.running = False
        if hasattr(self, 'pygame_thread'):
            self.pygame_thread.join(timeout=1.0)
        pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Check if pygame and joystick are available
        import pygame
        pygame.init()
        
        node = JoystickControllerNode()
        
        if node.gamepad:
            rclpy.spin(node)
        else:
            print("\n" + "="*60)
            print("NO GAMEPAD DETECTED!")
            print("="*60)
            print("Please check:")
            print("1. Bluetooth gamepad is powered on")
            print("2. Gamepad is paired with your computer")
            print("3. Run 'jstest /dev/input/js0' to test")
            print("4. Install pygame: pip3 install pygame")
            print("="*60)
            
    except ImportError:
        print("pygame not installed. Please install it:")
        print("pip3 install pygame")
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