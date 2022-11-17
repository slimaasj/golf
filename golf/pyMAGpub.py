import smbus
import rclpy
from numpy import array
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from transforms3d.euler import euler2quat as quaternion_from_euler

class PyMAGPub(Node):
    def __init__(self):
        super().__init__("pyMAGpub")

        pub_mag = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic='mag',
            qos_profile=1
        )

        magXmin = -2854
        magYmin = -541
        magZmin = -2020
        magXmax = 11
        magYmax = 2240
        magZmax = 988

        mag_msg = PoseWithCovarianceStamped()
        mag_msg.header.frame_id = self.declare_parameter('frame_header', 'base_mag').value

        bus = smbus.SMBus(1)

        #initialise the magnetometer
        bus.write_byte_data(0x1C,0x20, 0b11011100)        # Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
        bus.write_byte_data(0x1C,0x21, 0b00100000)        # +/- 8 gauss
        bus.write_byte_data(0x1C,0x22, 0b00000000)        # Continuous-conversion mode

        def readMAGx():
            mag_l = 0
            mag_h = 0
            
            mag_l = bus.read_byte_data(0x1C, 0x28)
            mag_h = bus.read_byte_data(0x1C, 0x29)

            mag_combined = (mag_l | mag_h <<8)
            return mag_combined  if mag_combined < 32768 else mag_combined - 65536

        def readMAGy():
            mag_l = 0
            mag_h = 0
            
            mag_l = bus.read_byte_data(0x1C, 0x2A)
            mag_h = bus.read_byte_data(0x1C, 0x2B)

            mag_combined = (mag_l | mag_h <<8)
            return mag_combined  if mag_combined < 32768 else mag_combined - 65536

        def readMAGz():
            mag_l = 0
            mag_h = 0
            
            mag_l = bus.read_byte_data(0x1C, 0x2C)
            mag_h = bus.read_byte_data(0x1C, 0x2D)

            mag_combined = (mag_l | mag_h <<8)
            return mag_combined  if mag_combined < 32768 else mag_combined - 65536


        #Read the accelerometer,gyroscope and magnetometer values
        MAGx = readMAGx()
        MAGy = readMAGy()
        MAGz = readMAGz()

        #Apply compass calibration
        MAGx -= (magXmin + magXmax) /2
        MAGy -= (magYmin + magYmax) /2
        MAGz -= (magZmin + magZmax) /2

        q = quaternion_from_euler(float(MAGx), float(MAGy), float(MAGz))
        mag_msg.pose.pose.position.x = 0.0
        mag_msg.pose.pose.position.y = 0.0
        mag_msg.pose.pose.position.z = 0.0
        mag_msg.pose.pose.orientation.x = q[0]
        mag_msg.pose.pose.orientation.y = q[1]
        mag_msg.pose.pose.orientation.z = q[2]
        mag_msg.pose.pose.orientation.w = q[3]
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        pub_mag.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PyMAGPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()