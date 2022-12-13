import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .submodules import md25_driver
import math

WHEEL_RADIUS = 0.099 #The Radius of the Wheels in Meters
WHEEL_SEPARATION =  0.285 #The Seperation of the Wheels in Meters

#Define global storage variables for encoder values
encoder_count_left = 0 
encoder_count_right = 0

def encoder_to_odometry(x, y, theta, left_count, right_count):
    #Compute the distance travveled by each wheel
    left_distance = left_count *2 * math.pi * WHEEL_RADIUS
    right_distance = right_count *2 * math.pi * WHEEL_RADIUS

    #Compute the delta in position and heading of the robot
    dx = (left_distance + right_distance) / 2
    dtheta = (right_distance - left_distance) / WHEEL_SEPARATION

    #Update the position and heading of the robot
    # Compute the new position and heading of the robot
    x += dx * math.cos(theta + dtheta / 2)
    y += dx * math.sin(theta + dtheta / 2)
    theta += dtheta

    # Return the updated values as a tuple
    return x, y, theta


def yaw_to_quaternion(yaw):
    qw = math.cos(yaw / 2)
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2)
    return (qw, qx, qy, qz)

#Credits: https://www.instructables.com/Joystick-to-Differential-Drive-Python/
def map(v, in_min, in_max, out_min, out_max):
    if v < in_min:
        v = in_min
    if v > in_max:
        v = in_max
    return (v - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def joystickToDiff(x, y, minJoystick, maxJoystick, minSpeed, maxSpeed):# If x and y are 0, then there is not much to calculate...
    if x == 0 and y == 0:
        return (128, 128)
    

    # First Compute the angle in deg
    # First hypotenuse
    z = math.sqrt(x * x + y * y)

    # angle in radians
    rad = math.acos(math.fabs(x) / z)

    # and in degrees
    angle = rad * 180 / math.pi
    
    #print(angle)
    # Now angle indicates the measure of turn
    # Along a straight line, with an angle o, the turn co-efficient is same
    # this applies for angles between 0-90, with angle 0 the coeff is -1
    # with angle 45, the co-efficient is 0 and with angle 90, it is 1

    tcoeff = -1 + (angle / 90) * 2
    turn = tcoeff * math.fabs(math.fabs(y) - math.fabs(x))
    turn = round(turn * 100, 0) / 100

    # And max of y or x is the movement
    mov = max(math.fabs(y), math.fabs(x))

    # First and third quadrant
    if (x >= 0 and y >= 0) or (x < 0 and y < 0):
        rawLeft = mov
        rawRight = turn
    else:
        rawRight = mov
        rawLeft = turn

    # Reverse polarity
    if y < 0:
        rawLeft = 0 - rawLeft
        rawRight = 0 - rawRight

    # minJoystick, maxJoystick, minSpeed, maxSpeed
    # Map the values onto the defined rang
    rightOut = map(rawRight, minJoystick, maxJoystick, minSpeed, maxSpeed)
    leftOut = map(rawLeft, minJoystick, maxJoystick, minSpeed, maxSpeed)

    #if y < 0:
    #    remember = leftOut
    #    leftOut = rightOut
    #    rightOut = remember
            
    return (rightOut, leftOut)


md = md25_driver.md25()

class DriverNode(Node):

    def __init__(self):
        super().__init__('driver_node')
        self.subscription_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        # Set initial Robot Position and heading if the Robot
        self.x = 0
        self.y = 0
        self.theta = 0
        # Store current cmd_vel
        self.msg = Twist()

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.odom_callback)

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        drive1 = 128
        drive2 = 128
        drive1, drive2 = joystickToDiff(angular, linear, -1, +1, 1, 255)
        
        md.drive(int(drive1), int(drive2)) #drives both motors at speed 100 using the default mode
        print("Drive: " + str(drive1) + " and " + str(drive2))
        self.msg = msg

    def odom_callback(self):
        # Convert encoder values to odom 
        left_count, right_count = md.encoders()
        print("Encoders: " + str(left_count) + " and " + str(right_count))
        print("Position Pre-Command: X: " + str(self.x) + " , Y: " + str(self.y) + " and Theta: " + str(self.theta))
        # Convert the encoder counts to odometry data
        self.x, self.y , self.theta = encoder_to_odometry(self.x, self.y, self.theta, left_count, right_count)
        print("Position Post-Command: X: " + str(self.x) + " , Y: " + str(self.y) + " and Theta: " + str(self.theta))

        # Create an Odometry message and fill it with data from the Twist message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        qw, qx, qy, qz = yaw_to_quaternion(self.theta)
        odom_msg.pose.pose.orientation.x = qw
        odom_msg.pose.pose.orientation.y = qx
        odom_msg.pose.pose.orientation.z = qy
        odom_msg.pose.pose.orientation.w = qz
        odom_msg.twist.twist = self.msg

        print(odom_msg)
        # Publish the Odometry message
        self.publisher_.publish(odom_msg)



def main(args=None):
    rclpy.init(args=args)

    odom_publisher = DriverNode()

    rclpy.spin(odom_publisher)

    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
