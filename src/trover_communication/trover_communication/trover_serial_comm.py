#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import serial
import serial.tools.list_ports
import math

class Trover_Serial_Node(Node):
    def __init__(self):
        super().__init__('Trover_Comm_Node')
        
        # Try to auto-detect Arduino port
        self.serial_port = None
        port = None
        
        if port is None:
            self.get_logger().warn('Arduino not auto-detected, trying /dev/ttyACM0')
            port = '/dev/ttyACM0'

        
        # Setup serial connection
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=1
            )
            self.get_logger().info(f'Serial port opened: {self.serial_port.name}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.get_logger().error('Node will run but serial communication disabled')
            self.serial_port = None
        
        # Declare publishers and subscribers
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Define a timer (20 Hz for robotics)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Define variables
        self.vx = 0.0
        self.omega = 0.0
    
    def find_arduino_port(self):
        """Auto-detect Arduino port"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(port)

            # Look for Arduino or common USB-serial chips
            if any(keyword in port.description.lower() for keyword in ['arduino', 'ch340', 'cp210', 'ftdi', 'atmel', 'atmega']):
                self.get_logger().info(f'Found Arduino at: {port.device}')
                return port.device
        return None
    
    def timer_callback(self):
        """Runs every time the timer triggers"""
        # Check if serial port exists and is open
        if self.serial_port is None or not self.serial_port.is_open:
            return
        
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline()
                
                # Handle decoding errors
                try:
                    decoded = line.decode('utf-8').strip()
                except UnicodeDecodeError:
                    self.get_logger().warn("Bad data received")
                    return
                
                # Handle parsing errors
                try:
                    values = decoded.split(',')
                    ###########RECEIVE OUR INFO##############    
                    vx = float(values[0])
                    omega = float(values[1])

                    #Add the IMU HERE once it's calibrated

                    self.publish_odom(vx, omega)
                    #########################################
                    #self.get_logger().info(f"vx: {vx}, omega: {omega}")
                except (ValueError, IndexError) as e:
                    test = 2
                    #self.get_logger().warn(f"Parse error: {e}")
                    
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            self.serial_port = None  # Mark as disconnected
    
    def publish_odom(self, vx, omega):
        """Publish odometry message"""
        msg = Odometry()
        
        # ===== HEADER (ALWAYS REQUIRED) =====
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # ===== VELOCITIES (WHAT WE ARE PROVIDING) =====
        msg.twist.twist.linear.x = vx       # Forward velocity (m/s)
        msg.twist.twist.linear.y = 0.0      # No lateral motion for diff drive
        msg.twist.twist.angular.z = omega   # Angular velocity (rad/s)
        
        # ===== VELOCITY COVARIANCE (HOW UNCERTAIN YOU ARE) =====
        msg.twist.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,   # vx variance
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,   # vy variance
            0.0,  0.0,  0.01, 0.0,  0.0,  0.0,   # vz variance
            0.0,  0.0,  0.0,  0.01, 0.0,  0.0,   # roll rate variance
            0.0,  0.0,  0.0,  0.0,  0.01, 0.0,   # pitch rate variance
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05   # yaw rate (omega) variance
        ]
        
        self.publisher.publish(msg)
    
    def cmd_vel_callback(self, msg):
        """Called whenever a cmd_vel message is received"""
        # Check if serial port exists
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn('Serial port not available, cannot send cmd_vel')
            return
        
        # Extract velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Format data to send (customize this for your protocol)
        data_string = f"{linear_x},{angular_z}\n"
        
        # Send over serial
        try:
            self.serial_port.write(data_string.encode('utf-8'))
            self.get_logger().info(f'Sent: {data_string.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.serial_port = None  # Mark as disconnected
    
    def destroy_node(self):
        """Clean up when node shuts down"""
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    serial_node = Trover_Serial_Node()
    
    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        pass
    finally:
        serial_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()