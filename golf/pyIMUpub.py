import smbus
import time
import datetime
import rclpy
from numpy import array
from rclpy.node import Node
from sensor_msgs.msg import Imu
from transforms3d.euler import euler2quat as quaternion_from_euler

class PyIMUPub(Node):
    def __init__(self):
        super().__init__("pyIMUpub")

        pub_imu = self.create_publisher(
            msg_type=Imu,
            topic='imu',
            qos_profile=1,
        )
        
        RAD_TO_DEG = 57.29578
        G_TO_MS2 = 9.80665
        G_GAIN = 0.070          # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly

        bus = smbus.SMBus(1)

        imu_msg = Imu()

        # TODO arrays not supported as parameter type ROS2
        imu_msg.orientation_covariance = [0.0025, 0.0, 0.0,
                                          0.0, 0.0025, 0.0,
                                          0.0, 0.0, 0.0025]
    
        imu_msg.angular_velocity_covariance = [0.002, 0.0, 0.0,
                                               0.0, 0.002, 0.0,
                                               0.0, 0.0, 0.002]
        
        imu_msg.linear_acceleration_covariance = [0.04, 0.0, 0.0,
                                                  0.0, 0.04, 0.0,
                                                  0.0, 0.0, 0.04]

        imu_msg.header.frame_id = self.declare_parameter('frame_header', 'base_imu_link').value

        #Initialise the IMU
        #initialise the accelerometer
        bus.write_byte_data(0x6A,0x10,0b10011111)         #ODR 3.33 kHz, +/- 8g , BW = 400hz
        bus.write_byte_data(0x6A,0x17,0b11001000)         #Low pass filter enabled, BW9, composite filter
        bus.write_byte_data(0x6A,0x12,0b01000100)         #Enable Block Data update, increment during multi byte read

        #initialise the gyroscope
        bus.write_byte_data(0x6A,0x11,0b10011100)         #ODR 3.3 kHz, 2000 dps

        def readACCx():
            acc_l = 0
            acc_h = 0

            acc_l = bus.read_byte_data(0x6A, 0x28)
            acc_h = bus.read_byte_data(0x6A, 0x29)

            acc_combined = (acc_l | acc_h <<8)
            return acc_combined  if acc_combined < 32768 else acc_combined - 65536


        def readACCy():
            acc_l = 0
            acc_h = 0
            
            acc_l = bus.read_byte_data(0x6A, 0x2A)
            acc_h = bus.read_byte_data(0x6A, 0x2B)

            acc_combined = (acc_l | acc_h <<8)
            return acc_combined  if acc_combined < 32768 else acc_combined - 65536


        def readACCz():
            acc_l = 0
            acc_h = 0
            
            acc_l = bus.read_byte_data(0x6A, 0x2C)
            acc_h = bus.read_byte_data(0x6A, 0x2D)

            acc_combined = (acc_l | acc_h <<8)
            return acc_combined  if acc_combined < 32768 else acc_combined - 65536


        def readGYRx():
            gyr_l = 0
            gyr_h = 0
            
            gyr_l = bus.read_byte_data(0x6A, 0x22)
            gyr_h = bus.read_byte_data(0x6A, 0x23)

            gyr_combined = (gyr_l | gyr_h <<8)
            return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536


        def readGYRy():
            gyr_l = 0
            gyr_h = 0
            
            gyr_l = bus.read_byte_data(0x6A, 0x24)
            gyr_h = bus.read_byte_data(0x6A, 0x25)

            gyr_combined = (gyr_l | gyr_h <<8)
            return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

        def readGYRz():
            gyr_l = 0
            gyr_h = 0
            
            gyr_l = bus.read_byte_data(0x6A, 0x26)
            gyr_h = bus.read_byte_data(0x6A, 0x27)

            gyr_combined = (gyr_l | gyr_h <<8)
            return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536


        while True:
            #Read and Convert Gyro raw to rad per second
            GYRx =  (readGYRx() * G_GAIN)/RAD_TO_DEG
            GYRy =  (readGYRy() * G_GAIN)/RAD_TO_DEG
            GYRz =  (readGYRz() * G_GAIN)/RAD_TO_DEG

            #Read and Convert Accel to m/s
            ACCx = ((readACCx() * 0.244)/1000) * G_TO_MS2
            ACCy = ((readACCy() * 0.244)/1000) * G_TO_MS2
            ACCz = ((readACCz() * 0.244)/1000) * G_TO_MS2

            if 1:       #Change to '0' to stop showing the angles from the accelerometer
                outputString += "#  ACCx %5.2f ACCy %5.2f ACCz %5.2f  #  " % (ACCx, ACCy, ACCz)

            if 1:       #Change to '0' to stop  showing the angles from the gyro
                outputString +="\t# GRYx %5.2f  GYRy %5.2f  GYRz %5.2f # " % (GYRx, GYRy, GYRz)

            print(outputString)

            imu_msg.linear_acceleration.x = float(ACCx)
            imu_msg.linear_acceleration.y = float(ACCy)
            imu_msg.linear_acceleration.z = float(ACCz)

            imu_msg.angular_velocity.x = float(GYRx)
            imu_msg.angular_velocity.y = float(GYRy)
            imu_msg.angular_velocity.z = float(GYRz)

            q = quaternion_from_euler(float(GYRx), float(GYRy), float(GYRz))
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]

            imu_msg.header.stamp = self.get_clock().now().to_msg()
            pub_imu.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PyIMUPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()