import sys
from robot_interfaces.srv import LogVelocity
import rclpy
from rclpy.node import Node


class VelocityLoggingClientAsync(Node):

    def __init__(self):
        super().__init__('velocity_logging_client_async')
        self.cli = self.create_client(LogVelocity, 'log_velocity')
        self.service_ready = False
        self.connection_attempts_limit = 3
        
        cnt = 0
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Connection to service failed, trying to connect again...')
            cnt += 1
            if cnt == self.connection_attempts_limit:
                break
            
        if self.cli.service_is_ready():
            self.service_ready = True 
                          
        self.req = LogVelocity.Request()

    def send_request(self):
        self.req.lvelocity = float(sys.argv[1])
        self.req.avelocity = float(sys.argv[2])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    client = VelocityLoggingClientAsync()
    
    if not client.service_ready:
        client.get_logger().info('Service is not available.')
    else:
        client.send_request()
    
        while rclpy.ok():
            rclpy.spin_once(client)
            
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    client.get_logger().info(
                        'Linear Velocity: %f; Angular Velocity: %f; LogID: %d; ProcID: %d' %
                        (client.req.lvelocity, client.req.avelocity, 
                         response.logid, response.procid))
                break
        

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



