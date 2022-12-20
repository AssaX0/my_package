import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

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

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


md = md25_driver.md25()

class OdometryNode(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('odometry_node')

        # Set the frame IDs and update rate
        self.frame_id = 'odom'
        self.child_frame_id = 'base_link'
        self.update_rate = 10  # Hz

        # Set the kinematic constants for the robot
        self.wheel_radius = 0.005  # meters
        self.wheel_base = 0.285  # meters
        self.encoder_resolution = (2 * pi * self.wheel_radius)  / 255  # meters per tick / ggf 250

        # Initialize the position and orientation of the robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.msg = Twist()

        # Create a subscriber for the cmd_vel topic
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Create a publisher for the odometry topic
        self.odometry_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Create a publisher for the joint_states topic
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
    
    def cmd_vel_callback(self, msg):

        linear = msg.linear.x
        angular = msg.angular.z
        drive1 = 128
        drive2 = 128
        drive1, drive2 = joystickToDiff(angular, linear, -1, +1, 1, 255)
        
        md.drive(int(drive1), int(drive2)) #drives both motors at speed 100 using the default mode
        #print("Drive: " + str(drive1) + " and " + str(drive2))
        self.msg = msg

        # Use the cmd_vel message to update the encoder values
        encoder_left, encoder_right = md.encoders()
        #print("Encoders Value: " + str(left_count) + " and " + str(right_count))
       
        # Calculate the odometry based on the encoder values
        odometry = self.calculate_odometry(encoder_left, encoder_right)
        
        # Publish the odometry message
        self.odometry_publisher.publish(odometry)
        
        # Create a JointState message with the joint positions of the wheels
        joint_state = JointState()
        joint_state.name = ['left_wheel', 'right_wheel']
        joint_state.position = [float(encoder_left), float(encoder_right)]
        self.joint_state_publisher.publish(joint_state)

    
    def calculate_odometry(self, encoder_left, encoder_right):
        # Calculate the distance traveled by each wheel
        distance_left = float(encoder_left) * self.encoder_resolution * self.wheel_radius
        distance_right = float(encoder_right) * self.encoder_resolution * self.wheel_radius

        # Calculate the distance traveled by the center of the robot
        distance = (distance_left + distance_right) / 2

        # Calculate the change in orientation of the robot
        delta_theta = (distance_right - distance_left) / self.wheel_base

        # Update the position and orientation of the robot based on the distance traveled and change in orientation
        self.x += float(distance * cos(self.theta))
        self.y += float(distance * sin(self.theta))
        self.theta += delta_theta

        # Create and return the odometry message
        odometry = Odometry()
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = self.frame_id
        odometry.child_frame_id = self.child_frame_id
        odometry.pose.pose.position.x = self.x
        odometry.pose.pose.position.y = self.y
        quaternion = euler_to_quaternion(0, 0, self.theta) # roll,pitch,yaw
        odometry.pose.pose.orientation.x = quaternion.x
        odometry.pose.pose.orientation.y = quaternion.y
        odometry.pose.pose.orientation.z = quaternion.z
        odometry.pose.pose.orientation.w = quaternion.w
        odometry.twist.twist.linear.x = distance / self.update_rate
        odometry.twist.twist.angular.z = delta_theta / self.update_rate
        return odometry

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()