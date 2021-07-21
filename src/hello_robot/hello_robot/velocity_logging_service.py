
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import LogVelocity
from hello_robot.settings import DB_FILE
from hello_robot.utils.dbops import create_connection, save_velocity_input, process_velocity_input

class VelocityLoggingService(Node):

    def __init__(self):
        super().__init__('velocity_logging_service')
        self.srv = self.create_service(LogVelocity, 'log_velocity', self.log_velocity_callback)
        self.conn = create_connection(DB_FILE)
        self.get_logger().info('Velocity Logging Service is ready:')


    def log_velocity_callback(self, request, response):
        logid_id = 0
        processing_id = 0
        lvelocity = float(request.lvelocity)
        avelocity = float(request.avelocity)
        
        with self.conn:
            log_id = save_velocity_input(self.conn, lvelocity, avelocity) 
            processing_id = process_velocity_input(self.conn, log_id, lvelocity, avelocity)
        
        response.logid = log_id
        response.procid = processing_id
        
        self.get_logger().info('Incoming request\nlvelocity: %f avelocity: %f' % (
            request.lvelocity, request.avelocity)
        )
        
        self.get_logger().info('Velocity Logging Service is ready:')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    service = VelocityLoggingService()
    rclpy.spin(service)
    if service.conn:
        service.conn.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    