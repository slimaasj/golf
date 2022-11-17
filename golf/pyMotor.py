import RPi.GPIO as GPIO
from time import sleep
import rclpy
import math
from rclpy.node import Node

class PyMotor(Node):
    def __init__(self):
        super().__init__("pyMotor")

        Motor_left_speed = 12		# PWM pin connected to Left Motor
        Motor_L_Dir = 31            # Bin pin connected to Left Motor
        Motor_right_speed = 13	    # PWM pin connected to Right Motor
        Motor_R_Dir = 16	        # Bin pin connected to Right Motor

        stop = 0
        forward = 1
        backward = 2
        right = 3
        left = 4

        # !ADD CODE FOR LISTENING TO ODOMETRY AND WAYPOINTS

        x = 0.0             # Self Location
        y = 0.0
        theta = 0.0

        goal_x = 0.0        # Next waypoint coordinates
        goal_y = 0.0

        TWOPI = 6.2831853071795865
        RAD2DEG = 57.2957795130823209

        GPIO.setwarnings(False)			            #disable warnings
        GPIO.setmode(GPIO.BOARD)		            #set pin numbering system
        GPIO.setup(Motor_left_speed,GPIO.OUT)
        GPIO.setup(Motor_L_Dir,GPIO.OUT)
        GPIO.setup(Motor_right_speed,GPIO.OUT)
        GPIO.setup(Motor_R_Dir,GPIO.OUT)

        Motor_L_Spd = GPIO.PWM(Motor_left_speed,1000)		#create PWM instance with frequency
        Motor_R_Spd = GPIO.PWM(Motor_right_speed,1000)		#create PWM instance with frequency
        Motor_L_Spd.start(0)				                #start PWM of required Duty Cycle 
        Motor_R_Spd.start(0)				                #start PWM of required Duty Cycle 

        def motor_control(direction):       # !ADD GRADUAL SPEED CHANGE!
            if (direction == stop):
                Motor_L_Spd.ChangeDutyCycle(0)
                Motor_R_Spd.ChangeDutyCycle(0)
                GPIO.output(Motor_L_Dir, GPIO.HIGH)
                GPIO.output(Motor_R_Dir, GPIO.HIGH)

            elif (direction == forward):
                Motor_L_Spd.ChangeDutyCycle(100)
                Motor_R_Spd.ChangeDutyCycle(100)
                GPIO.output(Motor_L_Dir, GPIO.HIGH)
                GPIO.output(Motor_R_Dir, GPIO.HIGH)

            elif (direction == backward):
                Motor_L_Spd.ChangeDutyCycle(100)
                Motor_R_Spd.ChangeDutyCycle(100)
                GPIO.output(Motor_L_Dir, GPIO.LOW)
                GPIO.output(Motor_R_Dir, GPIO.LOW)

            elif (direction == right):
                Motor_L_Spd.ChangeDutyCycle(100)
                Motor_R_Spd.ChangeDutyCycle(100)
                GPIO.output(Motor_L_Dir, GPIO.HIGH)
                GPIO.output(Motor_R_Dir, GPIO.LOW)

            elif (direction == left):
                Motor_L_Spd.ChangeDutyCycle(100)
                Motor_R_Spd.ChangeDutyCycle(100)
                GPIO.output(Motor_L_Dir, GPIO.LOW)
                GPIO.output(Motor_R_Dir, GPIO.HIGH)

        def goal_theta():
            if (x == goal_x and y == goal_y):
                return 0
            else:
                calc_theta = math.atan2(x - goal_x, y - goal_y)
                if (calc_theta < 0.0):
                    calc_theta += TWOPI

                return RAD2DEG * calc_theta

        if (theta > (goal_theta()+1) or theta < (goal_theta()-1)):      #First, check to see if within +/- 1 degree of goal heading
            if (theta > (goal_theta()+1)):
                motor_control(left)

            elif (theta < (goal_theta()-1)):
                motor_control(right)

        elif (x > (goal_x + 0.1) or x < (goal_x - 0.1), y > (goal_y + 0.1), y < (goal_y - 0.1)): 
            motor_control(forward)

        else:
            motor_control(stop)

def main(args=None):
    rclpy.init(args=args)
    node = PyMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()