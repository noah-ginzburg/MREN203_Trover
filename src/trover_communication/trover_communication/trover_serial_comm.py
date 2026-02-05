import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import serial
import math

class Trover_Serial_Node(Node):

    def __init__(self):
        super().__init__('Trover_Comm_Node')

        # Setup serial connection
        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyUSB0',        # Change to your port
                baudrate=115200,             # Match your device
                timeout=1
            )
            self.get_logger().info(f'Serial port opened: {self.serial_port.name}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        #Declare publishers and Subscribers
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.subcriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        #Define a timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Define variables
        self.vx = 0.0
        self.omega = 0.0

    def timer_callback(self):
        """Runs every time the timer triggers after 'timer_period' """
        try:
            if self.serial_port.in_waiting > 0:
                line = ser.readline()
                
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

                    self.publish_odom(vx, omega)
                    #########################################

                    self.get_logger().info(f"vx: {vx}, omega: {omega}")
                except (ValueError, IndexError) as e:
                    self.get_logger().warn(f"Parse error: {e}")
                    
        except serial.SerialException as e:
            self.get_logger().warn(f"Serial error: {e}")
            return
        
    def publish_odom(self, vx, omega):

        msg = Odometry()

        # ===== HEADER (ALWAYS REQUIRED) =====
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # ===== VELOCITIES (WHAT WE ARE PROVIDING) =====
        msg.twist.twist.linear.x = vx      # Forward velocity (m/s)
        msg.twist.twist.linear.y = 0.0          # No lateral motion for diff drive
        msg.twist.twist.angular.z = omega  # Angular velocity (rad/s)
        
        # ===== VELOCITY COVARIANCE (HOW UNCERTAIN YOU ARE) =====
        msg.twist.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,   # vx variance
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,   # vy variance
            0.0,  0.0,  0.01, 0.0,  0.0,  0.0,   # vz variance
            0.0,  0.0,  0.0,  0.01, 0.0,  0.0,   # roll rate variance
            0.0,  0.0,  0.0,  0.0,  0.01, 0.0,   # pitch rate variance
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05   # yaw rate (omega) variance
        ]

        #VarianceStandard DeviationMeaning0.001±0.03 m/sVery accurate (high-quality encoders)0.01±0.1 m/sGood (typical for wheel encoders)0.1±0.32 m/sModerate (decent odometry)0.5±0.71 m/sPoor (estimated velocities)1.0±1.0 m/sVery poor (barely useful)
        
        #For angular velocity (omega):
        #VarianceStandard DeviationMeaning0.01±0.1 rad/s (±5.7°/s)Good0.05±0.22 rad/s (±12.7°/s)Typical0.1±0.32 rad/s (±18°/s)Moderate0.5±0.71 rad/s (±40°/s)Poor

        self.publisher.publish(msg)

    def cmd_vel_callback(self, msg):
        """Called whenever a cmd_vel message is received"""
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
    
    def destroy_node(self):
        """Clean up when node shuts down"""
        if self.serial_port.is_open:
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
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        serial_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()