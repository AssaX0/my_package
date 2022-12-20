from math import sin, cos, pi, acos, sqrt, fabs
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from .submodules import md25_driver
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

WHEEL_RADIUS = 0.05 #The Radius of the Wheels in Meters
WHEEL_SEPARATION =  0.285 #The Seperation of the Wheels in Meters

def encoder_to_odometry(x, y, theta, left_count, right_count):
    #Compute the distance travveled by each wheel
    left_distance = left_count *2 * pi * WHEEL_RADIUS
    right_distance = right_count *2 * pi * WHEEL_RADIUS

    #Compute the delta in position and heading of the robot
    dx = (left_distance + right_distance) / 2
    dtheta = (right_distance - left_distance) / WHEEL_SEPARATION

    #Update the position and heading of the robot
    # Compute the new position and heading of the robot
    x += dx * cos(theta + dtheta / 2)
    y += dx * sin(theta + dtheta / 2)
    theta += dtheta

    # Return the updated values as a tuple
    return x, y, theta


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

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
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odometry_publisher = self.create_publisher(Odometry, 'odometry', 10)
        self.broadcaster = TransformBroadcaster(self, qos=10)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        
        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # Set initial Robot Position and heading if the Robot
        self.x = 0
        self.y = 0
        self.theta = 0
        # Store current cmd_vel
        self.msg = Twist()
        #Store last encoder values
        #elf.encoder_count_left = 0 
        #self.encoder_count_right = 0

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.odom_callback)

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        drive1 = 128
        drive2 = 128
        drive1, drive2 = joystickToDiff(angular, linear, -1, +1, 1, 255)
        
        md.drive(int(drive1), int(drive2)) #drives both motors at speed 100 using the default mode
        #print("Drive: " + str(drive1) + " and " + str(drive2))
        self.msg = msg

    def odom_callback(self):
        # Convert encoder values to odom 
        left_count, right_count = md.encoder_diff()
        #print("Encoders Value: " + str(left_count) + " and " + str(right_count))

        # Save Value and Calculate diff
        #diff_left , diff_right = left_count - self.encoder_count_left, right_count - self.encoder_count_right
        #self.encoder_count_left, self.encoder_count_right = left_count, right_count
        #print("Encoder Diff: " + str(diff_left) + " and " + str(diff_right))

        left_rev, right_rev = float(left_count)/350 , float(right_count) / 350
        #print("Revolutions: " + str(left_rev) + " and " + str(right_rev))
        
        #print("Position Pre-Command: X: " + str(self.x) + " , Y: " + str(self.y) + " and Theta: " + str(self.theta))
        # Convert the encoder counts to odometry data
        self.x, self.y , self.theta = encoder_to_odometry(self.x, self.y, self.theta, left_rev, right_rev)
        #print("Position Post-Command: X: " + str(self.x) + " , Y: " + str(self.y) + " and Theta: " + str(self.theta))

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        joint_state = JointState()

        # get wheel states
        left_state, right_state = md.motor_state()
        #left_state, right_state = left_count, right_count 
        while left_state > 350:
            left_state = left_state - 350

        while right_state > 350:
            right_state = right_state - 350

        left_state, right_state = float(left_state)/350*pi*2, float(right_state)/350*pi*2
        #print("State- Position ... Left: " + str(left_state) + " , Right: " + str(right_state))

        # update joint_state
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [left_state, right_state]
        #joint_state.velocity = [0.0, 0.0]
        #joint_state.effort = [0.0, 0.0]
       

        # update transform
        odom_trans.header.stamp = now.to_msg()
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = euler_to_quaternion(0, 0, self.theta) # roll,pitch,yaw

        # send the joint state and transform
        #self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(odom_trans)


        # Create an Odometry message and fill it with data from the Twist message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        q = euler_to_quaternion(0,0, self.theta)
        #qw, qx, qy, qz = tf2_ros.transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q.z
        odom_msg.pose.pose.orientation.y = q.y
        odom_msg.pose.pose.orientation.z = q.z
        odom_msg.pose.pose.orientation.w = q.w
        odom_msg.twist.twist = self.msg

        self.odometry_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)

    odom_publisher = DriverNode()

    rclpy.spin(odom_publisher)

    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
