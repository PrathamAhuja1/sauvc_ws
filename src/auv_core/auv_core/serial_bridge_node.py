# auv_core/serial_bridge_node.py

import rclpy
from rclpy.node import Node
import serial
import time
import threading
from std_msgs.msg import Float32 # For depth
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3Stamped, QuaternionStamped # For commands and maybe raw IMU data
from rclpy.qos import qos_profile_sensor_data # Use sensor profile for IMU/Depth out
import math

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.declare_parameter('serial_port', '/dev/ttyACM0') # Adjust as needed
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('imu_frame_id', 'imu_link')

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.imu_frame = self.get_parameter('imu_frame_id').value

        self.serial_port = None
        self.lock = threading.Lock() # Lock for serial port access

        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.05) # Shorter timeout
            self.get_logger().info(f"Attempting to open serial port {port} at {baud} baud.")
            time.sleep(2.0) # Give RP2040 time to reset after port opens
            if self.serial_port.is_open:
                self.serial_port.flushInput() # Clear any garbage data
                self.serial_port.flushOutput()
                self.get_logger().info(f"Successfully opened serial port {port}.")
            else:
                 raise serial.SerialException("Port is not open after Serial()")

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            rclpy.shutdown()
            return
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred opening serial port: {e}")
            rclpy.shutdown()
            return


        # --- Publishers (Data FROM RP2040) ---
        self.depth_pub = self.create_publisher(Float32, '/auv/raw_depth', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile=qos_profile_sensor_data)
        # Add other publishers as needed (e.g., battery voltage, raw accel/gyro)

        # --- Subscribers (Commands TO RP2040) ---
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/rp2040/cmd_vel', # INPUT: Topic from control_node
            self.cmd_vel_callback,
            10)
        # Add other subscribers (e.g., actuator commands)

        self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.read_thread.start()

        self.get_logger().info('Serial Bridge Node started.')

    def cmd_vel_callback(self, msg: Twist):
        # Format Twist message into serial command string
        # Example format: "CMD:VX=0.1;VY=0.0;VZ=0.0;RR=0.0;PR=0.0;YR=0.2\n"
        # Ensure values have reasonable precision to avoid overly long strings
        cmd_str = (f"CMD:VX={msg.linear.x:.3f};VY={msg.linear.y:.3f};VZ={msg.linear.z:.3f};"
                   f"RR={msg.angular.x:.3f};PR={msg.angular.y:.3f};YR={msg.angular.z:.3f}\n")
        self.send_serial_command(cmd_str)

    def send_serial_command(self, command):
        with self.lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    # self.get_logger().debug(f"Sending: {command.strip()}")
                    bytes_written = self.serial_port.write(command.encode('utf-8'))
                    # self.get_logger().debug(f"Wrote {bytes_written} bytes.")
                except serial.SerialTimeoutException:
                     self.get_logger().warn("Serial write timeout.")
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial write error: {e}")
                    # Consider trying to reopen the port or shutting down
                except Exception as e:
                     self.get_logger().error(f"Error sending serial command: {e}")
            else:
                self.get_logger().warn("Serial port not available for writing.")


    def read_serial_data(self):
        buffer = ""
        while rclpy.ok():
            line = None
            with self.lock:
                if not self.serial_port or not self.serial_port.is_open:
                    time.sleep(0.5) # Wait if port closed
                    continue
                try:
                    # Read line with timeout
                    line_bytes = self.serial_port.readline()
                    if line_bytes:
                        line = line_bytes.decode('utf-8', errors='ignore').strip()

                except serial.SerialException as e:
                    self.get_logger().error(f"Serial read error: {e}")
                    # Attempt to close and reopen? For now, just log and wait.
                    try:
                        self.serial_port.close()
                    except: pass
                    self.serial_port = None # Signal port is closed
                    time.sleep(2.0) # Wait before potential reconnection attempt (not implemented here)
                    continue # Skip processing this loop
                except Exception as e:
                     self.get_logger().error(f"Error reading serial line: {e}")

            # Process the line outside the lock to avoid holding it during parsing/publishing
            if line:
                 self.process_serial_line(line)
            else:
                # No data read or timeout occurred, sleep briefly
                time.sleep(0.005)


    def process_serial_line(self, line):
         if not line: return
         # self.get_logger().debug(f"Received: {line}")
         try:
            if line.startswith("DATA:"):
                parts = line[5:].split(';')
                data_dict = {}
                for part in parts:
                    # Ensure part is valid key=value pair
                    if '=' in part and len(part.split('=')) == 2:
                        key, value_str = part.split('=', 1)
                        key = key.strip()
                        value_str = value_str.strip()
                        if key and value_str: # Make sure key and value are not empty
                            try:
                                data_dict[key] = float(value_str)
                            except ValueError:
                                self.get_logger().warn(f"Could not convert value '{value_str}' for key '{key}' to float.")
                                return # Skip this line if any part fails parsing

                now = self.get_clock().now().to_msg()

                # Publish Depth
                if 'DEPTH' in data_dict:
                    depth_msg = Float32()
                    depth_msg.data = data_dict['DEPTH']
                    self.depth_pub.publish(depth_msg)

                # Publish IMU
                imu_fields_minimal = ['IMU_AX', 'IMU_AY', 'IMU_AZ', 'IMU_GX', 'IMU_GY', 'IMU_GZ']
                orientation_fields = ['IMU_QW', 'IMU_QX', 'IMU_QY', 'IMU_QZ']
                has_minimal = all(field in data_dict for field in imu_fields_minimal)
                has_orientation = all(field in data_dict for field in orientation_fields)

                if has_minimal:
                    imu_msg = Imu()
                    imu_msg.header.stamp = now
                    imu_msg.header.frame_id = self.imu_frame

                    # Linear Acceleration (m/s^2)
                    imu_msg.linear_acceleration.x = data_dict['IMU_AX']
                    imu_msg.linear_acceleration.y = data_dict['IMU_AY']
                    imu_msg.linear_acceleration.z = data_dict['IMU_AZ']
                    # Set covariance (TUNE based on IMU specs/testing)
                    imu_msg.linear_acceleration_covariance[0] = 0.01 # Example variance X
                    imu_msg.linear_acceleration_covariance[4] = 0.01 # Example variance Y
                    imu_msg.linear_acceleration_covariance[8] = 0.01 # Example variance Z

                    # Angular Velocity (rad/s) - MAKE SURE RP2040 SENDS RAD/S
                    imu_msg.angular_velocity.x = data_dict['IMU_GX']
                    imu_msg.angular_velocity.y = data_dict['IMU_GY']
                    imu_msg.angular_velocity.z = data_dict['IMU_GZ']
                    # Set covariance (TUNE based on IMU specs/testing)
                    imu_msg.angular_velocity_covariance[0] = 0.005 # Example variance Roll Rate
                    imu_msg.angular_velocity_covariance[4] = 0.005 # Example variance Pitch Rate
                    imu_msg.angular_velocity_covariance[8] = 0.005 # Example variance Yaw Rate

                    if has_orientation:
                        # Normalize quaternion just in case
                        qw, qx, qy, qz = data_dict['IMU_QW'], data_dict['IMU_QX'], data_dict['IMU_QY'], data_dict['IMU_QZ']
                        norm = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
                        if norm > 1e-6: # Avoid division by zero
                            imu_msg.orientation.w = qw / norm
                            imu_msg.orientation.x = qx / norm
                            imu_msg.orientation.y = qy / norm
                            imu_msg.orientation.z = qz / norm
                        else:
                            # Invalid quaternion, set to identity or skip?
                            imu_msg.orientation.w = 1.0
                            imu_msg.orientation.x = 0.0
                            imu_msg.orientation.y = 0.0
                            imu_msg.orientation.z = 0.0
                            self.get_logger().warn("Received near-zero quaternion from serial, setting to identity.")

                        # Set covariance (TUNE based on IMU specs/testing)
                        imu_msg.orientation_covariance[0] = 0.002 # Example variance Roll
                        imu_msg.orientation_covariance[4] = 0.002 # Example variance Pitch
                        imu_msg.orientation_covariance[8] = 0.005 # Example variance Yaw (Higher uncertainty)
                    else:
                        # No orientation provided, set covariance to -1 for first element
                        imu_msg.orientation_covariance[0] = -1.0

                    self.imu_pub.publish(imu_msg)

         except Exception as e:
             self.get_logger().error(f"Error processing line '{line}': {e}", exc_info=True)


    def destroy_node(self):
        self.get_logger().info("Shutting down Serial Bridge Node.")
        # Attempt to cleanly close the serial port
        with self.lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                    self.get_logger().info("Closed serial port.")
                except Exception as e:
                    self.get_logger().error(f"Error closing serial port: {e}")
            self.serial_port = None # Ensure thread stops trying to use it
        # Wait briefly for the thread to potentially exit based on rclpy.ok()
        if hasattr(self, 'read_thread') and self.read_thread.is_alive():
             self.read_thread.join(timeout=0.5)
             if self.read_thread.is_alive():
                  self.get_logger().warn("Read thread did not exit cleanly.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    if node.serial_port: # Only spin if port opened successfully
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('KeyboardInterrupt, shutting down.')
        except Exception as e:
            node.get_logger().error(f"Unhandled exception in spin: {e}")
        finally:
            # Explicitly destroy node to trigger cleanup
             if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()
    else:
        # Port didn't open, just shutdown
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()