
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CommandDemoSubscriber(Node):

    def __init__(self):
        super().__init__('command_subscriber')
        self.subscriber_ = self.create_subscription(Twist, 'turtle1/cmd_vel', self.subscribe_message, 1)
        self.subscriber_  # prevent unused variable warning

    def subscribe_message(self, msg):
        self.get_logger().info('Recieved - Linear Velocity : %f, Angular Velocity : %f' % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    command_subscriber = CommandDemoSubscriber()
    rclpy.spin(command_subscriber)
    command_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()