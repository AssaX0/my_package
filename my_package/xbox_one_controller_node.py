import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .submodules import xbox_one_controller_driver

controller = xbox_one_controller_driver.xboxcontroller()


class ControllerPublisher(Node):
    
    
    def __init__(self):
        super().__init__('controller_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.old = False

    def timer_callback(self):
        
        new = controller.readneeded()
        #if new != self.old:
        #self.old = new
        # Tausch
        y = float(new[0])
        x = float(new[1])
            
        #if x < 0:
        #    y = remap(y, -1, +1, +1, -1)

        #Ausgabe
        if new != self.old:
            self.old = new
            print("X Value: " + str(x) + " --- Y Value:" + str(y))
        
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = y
        #print(msg)
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ControllerPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
