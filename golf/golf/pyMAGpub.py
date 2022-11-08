import smbus
import time
import datetime
import rclpy
from numpy import array
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from transforms3d.euler import euler2quat as quaternion_from_euler

class PyMAGPub(Node):
    def __init__(self):
        super().__init__("pyMAGpub")

        pub_mag = self.create_publisher(
            msg_type=MagneticField,
            topic='mag',
            qos_profile=1,
        )

        magXmin =  0
        magYmin =  0
        magZmin =  0
        magXmax =  0
        magYmax =  0
        magZmax =  0

        bus = smbus.SMBus(1)

        outputString = ""

        mag_msg = MagneticField()

        # TODO arrays not supported as parameter type ROS2
        mag_msg.magnetic_field_covariance = [0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0]

        mag_msg.header.frame_id = self.declare_parameter('frame_header', 'base_mag_link').value

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


        while True:

            #Read the accelerometer,gyroscope and magnetometer values
            MAGx = readMAGx()
            MAGy = readMAGy()
            MAGz = readMAGz()


            #Apply compass calibration
            MAGx -= (magXmin + magXmax) /2
            MAGy -= (magYmin + magYmax) /2
            MAGz -= (magZmin + magZmax) /2


            if 1:       #Change to '0' to stop  showing the angles from the magnetometer
                outputString +="\t# MAGx %5.2f  MAGy %5.2f  MAGz %5.2f # " % (MAGx, MAGy, MAGz)

            print(outputString)

            mag_msg.magnetic_field.x = MAGx
            mag_msg.magnetic_field.y = MAGy
            mag_msg.magnetic_field.z = MAGz


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