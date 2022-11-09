import rclpy
from rclpy.node import Node
import pynmea2 as nmea
import time
import smbus
import signal
import sys
BUS = None
address = 0x42
gpsReadInterval = 0.03

from sensor_msgs.msg import NavSatFix

class PyGPSPub(Node):
    def __init__(self):
        super().__init__("pyGPSpub")
        
        pub_gps = self.create_publisher(
            msg_type=NavSatFix,
            topic='gps',
            qos_profile=10,
        )

        
        def connectBus():
            global BUS
            BUS = smbus.SMBus(1)

        def parseResponse(gpsLine):
            if(gpsLine.count(36) == 1):                                 # Check #1, make sure '$' doesnt appear twice
                if len(gpsLine) < 84:                                   # Check #2, 83 is maximun NMEA sentenace length.
                    CharError = 0;
                    for c in gpsLine:                                   # Check #3, Make sure that only readiable ASCII charaters and Carriage Return are seen.
                        if (c < 32 or c > 122) and  c != 13:
                            CharError+=1
                    if (CharError == 0):                                # Only proceed if there are no errors.
                        gpsChars = ''.join(chr(c) for c in gpsLine)
                        if (gpsChars.find('txbuf') == -1):              # Check #4, skip txbuff allocation error
                            gpsStr, chkSum = gpsChars.split('*',2)      # Check #5 only split twice to avoid unpack error
                            gpsComponents = gpsStr.split(',')
                            chkVal = 0
                            for ch in gpsStr[1:]:                       # Remove the $ and do a manual checksum on the rest of the NMEA sentence
                                chkVal ^= ord(ch)
                            if (chkVal == int(chkSum, 16)):             # Compare the calculated checksum with the one in the NMEA sentence
                                gpstimer_callback(gpsChars)

        def readGPS():
            c = None
            response = []
            try:
                while True: # Newline, or bad char.
                    c = BUS.read_byte(address)
                    if c == 255:
                        return False
                    elif c == 10:
                        break
                    else:
                        response.append(c)
                parseResponse(response)
            except IOError:
                connectBus()
            except Exception as e:
                print (e)
        connectBus()

        def gpstimer_callback(line):
            try:
                if "$GPGGA" in line:
                    msg = nmea.parse(line)
                    print(repr(msg))
                    # create navsat msg
                    gps_msg = NavSatFix()
                    gps_msg.altitude = float(msg.altitude)
                    gps_msg.latitude = float(msg.latitude)
                    gps_msg.longitude = float(msg.longitude)
                    #node.get_logger().info('Publishing gps data - Altitude: "%s" Latitude: "%s" Longitude: "%s" '%msg.altitude, msg.lat, msg.lon)
                    pub_gps.publish(gps_msg)     
            except nmea.ParseError as e:
                print (e)
                

        while True:
            readGPS()
            time.sleep(gpsReadInterval)

def main(args=None):
    rclpy.init(args=args)
    node = PyGPSPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()