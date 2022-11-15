import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

dummy = True
try:
    import smbus

    dummy = False
    print('smbus is available')
except:
    print('smbus not available; in dummy mode')

import time
import math

MD25_DEFAULT_ADDRESS = 0x58
MD25_DEFAULT_MODE = 0

MD25_REGISTER_SPEED1 = 0x00
MD25_REGISTER_SPEED2_TURN = 0x01
MD25_REGISTER_ENC1A = 0x02
MD25_REGISTER_ENC1B = 0x03
MD25_REGISTER_ENC1C = 0x04
MD25_REGISTER_ENC1D = 0x05
MD25_REGISTER_ENC2A = 0x06
MD25_REGISTER_ENC2B = 0x07
MD25_REGISTER_ENC2C = 0x08
MD25_REGISTER_ENC2D = 0x09
MD25_REGISTER_BATTERY_VOLTS = 0x0A
MD25_REGISTER_MOTOR1_CURRENT = 0x0B
MD25_REGISTER_MOTOR2_CURRENT = 0x0C
MD25_REGISTER_SOFTWARE_REV = 0x0D
MD25_REGISTER_ACCELERATION_RATE = 0x0E
MD25_REGISTER_MODE = 0x0F
MD25_REGISTER_COMMAND = 0x10


class md25:
    def __init__(self, mode=MD25_DEFAULT_MODE, bus=1, address=MD25_DEFAULT_ADDRESS):
        self.mode = mode
        self.address = address
        self.bus = None
        #print('dummy is', dummy)
        if not dummy:
            #print('setting up SMBus')
            self.bus = smbus.SMBus(bus)
            self.bus.write_byte_data(self.address, MD25_REGISTER_MODE, self.mode)

    def ensureSet(self, args, message='', all=True):
        for name in args:
            if None == args[name]:
                if all:
                    raise ValueError("%s was not set. %s" % (name, message));
            else:
                return
        raise ValueError("one of %s should be set. %s" % (args.keys(), message));

    def ensureRange(self, range, args, message=''):
        for name in args:
            if args[name] and (args[name] < range[0] or args[name] > range[1]):
                raise ValueError(
                    "%s (%i) was out of range (%i - %i). %s" % (name, args[name], range[0], range[1], message))

    def drive(self, motor0=None, motor1=None, speed=None, turn=None):
        #print(motor0, motor1, speed, turn)
        if 0 == self.mode:
            self.ensureSet({'motor0': motor0, 'motor1': motor1}, all=False)
            self.ensureRange((1, 255), {'motor0': motor0, 'motor1': motor1})
        if 1 == self.mode:
            self.ensureSet({'motor0': motor0, 'motor1': motor1}, all=False)
            self.ensureRange((-128, 127), {'motor0': motor0, 'motor1': motor1})
        if (0 == self.mode or 1 == self.mode) and self.bus:
            if motor0:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, motor0)
            if motor1:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, motor1)
        if 2 == self.mode:
            self.ensureSet({'speed': speed, 'turn': turn}, all=False)
            self.ensureRange((1, 255), {'speed': speed, 'turn': turn})
        if 3 == self.mode:
            self.ensureSet({'speed': speed, 'turn': turn}, all=False)
            self.ensureRange((-128, 127), {'speed': speed, 'turn': turn})
        if (2 == self.mode or 3 == self.mode) and self.bus:
            if speed:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, speed)
            if turn:
                self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, turn)

    def stop(self):
        #print('STOP!!!')
        if (0 == self.mode or 2 == self.mode) and self.bus:
            self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, 128)
            self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, 128)
        if (1 == self.mode or 3 == self.mode) and self.bus:
            self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED1, 0)
            self.bus.write_byte_data(self.address, MD25_REGISTER_SPEED2_TURN, 0)

    def battery(self):
        if self.bus:
            return self.bus.read_byte_data(self.address, MD25_REGISTER_BATTERY_VOLTS)
        else:
            return 120

#Credits: https://www.instructables.com/Joystick-to-Differential-Drive-Python/
def map(v, in_min, in_max, out_min, out_max):
    # Check that the value is at least in_min
    if v < in_min:
        v = in_min
    # Check that the value is at most in_max
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

    if y < 0:
        remember = leftOut
        leftOut = rightOut
        rightOut = remember
            
    return (rightOut, leftOut)


md = md25(0)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('driver_subscriber')
        self.subscription = self.create_subscription(Twist,'joy',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.y
        drive1 = 128
        drive2 = 128
        drive1, drive2 = joystickToDiff(angular, linear, -1, +1, 1, 255)
        
        print(drive1, drive2)
        md.drive(int(drive2), int(drive1)) #drives both motors at speed 100 using the default mode
        
        #self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    



