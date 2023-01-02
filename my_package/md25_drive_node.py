import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

from .submodules import md25_driver
from math import sin, cos, sqrt, acos, fabs, pi


#Credits: https://www.instructables.com/Joystick-to-Differential-Drive-Python/
# Renamed map to remap
def remap(v, in_min, in_max, out_min, out_max):
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
    z = sqrt(x * x + y * y)

    # angle in radians
    rad = acos(fabs(x) / z)

    # and in degrees
    angle = rad * 180 / pi
    
    #print(angle)
    # Now angle indicates the measure of turn
    # Along a straight line, with an angle o, the turn co-efficient is same
    # this applies for angles between 0-90, with angle 0 the coeff is -1
    # with angle 45, the co-efficient is 0 and with angle 90, it is 1

    tcoeff = -1 + (angle / 90) * 2
    turn = tcoeff * fabs(fabs(y) - fabs(x))
    turn = round(turn * 100, 0) / 100

    # And max of y or x is the movement
    mov = max(fabs(y), fabs(x))

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
    rightOut = remap(rawRight, minJoystick, maxJoystick, minSpeed, maxSpeed)
    leftOut = remap(rawLeft, minJoystick, maxJoystick, minSpeed, maxSpeed)

    #if y < 0:
    #    remember = leftOut
    #    leftOut = rightOut
    #    rightOut = remember
            
    return (rightOut, leftOut)


md = md25_driver.md25()

class OdometryNode(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('odometry_node')

        # Create a subscriber for the cmd_vel topic
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Create a publisher for the odometry topic
        self.encoder_left_publisher = self.create_publisher(Int32, 'encoder_left', 10)
        self.encoder_right_publisher = self.create_publisher(Int32, 'encoder_right', 10)

    
    def cmd_vel_callback(self, msg):

        linear = msg.linear.x
        angular = msg.angular.z
        drive1 = 128
        drive2 = 128
        drive1, drive2 = joystickToDiff(angular, linear, -1, +1, 1, 255)
        
        md.drive(int(drive1), int(drive2)) #drives both motors at speed 100 using the default mode
        #print("Drive: " + str(drive1) + " and " + str(drive2))

        encoder_left, encoder_right = md.encoders()

        # Publish the odometry message
        self.encoder_left_publisher.publish(encoder_left)
        self.encoder_right_publisher.publish(encoder_right)
        


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()