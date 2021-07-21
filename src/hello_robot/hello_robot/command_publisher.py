import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from hello_robot.settings import DB_FILE
from hello_robot.utils.dbops import create_connection
from hello_robot.utils.dbops import get_min_logid, get_command, delete_command
from hello_robot.utils.dbops import delete_all


class CommandPublisher(Node):

    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)
        timer_period = 5  # seconds
        
        self.conn = create_connection(DB_FILE)
        
        self.timer = self.create_timer(timer_period, self.publish_command)
        
        description = """This parameter allows to empty command list before launching the service,
            in order to do this you need to set reset to True. The default reset value is False.
        """ 
        
        # adding --reset option
        RESET = False
        parser = argparse.ArgumentParser()
        parser.add_argument('-r', '--reset', 
                            help='Reset command processing list by removing all existing commands',
                            action='store_true')
        args = parser.parse_args()
        if args.reset:
            self.get_logger().info('Reseting command processing list.')
            delete_all(self.conn)
            self.get_logger().info('All existing commands removed.')
        

    def publish_command(self):
        lvelocity = None
        avelocity = None
    
        # read command from db
        command_id = get_min_logid(self.conn)
        if command_id > 0:
            command = get_command(self.conn, command_id)
            lvelocity, avelocity = command[1], command[2]
            
        if lvelocity is not None and avelocity is not None:
            message = Twist()
            message.linear.x = lvelocity
            message.angular.z = avelocity
            self.get_logger().info('Sending - Linear Velocity : %f, Angular Velocity : %f' % 
                                       (message.linear.x, message.angular.z)
                                   )
            self.publisher_.publish(message)
            # delete processed command from db
            delete_command(self.conn, command_id)
            self.conn.commit()
        else:
            self.get_logger().info('...waiting for new commands...')
            

def main(args=None):
    rclpy.init(args=args)
    command_publisher = CommandPublisher()
    rclpy.spin(command_publisher)
    if command_publisher.conn:
        command_publisher.conn.close()
    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    