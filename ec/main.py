import rclpy
import os
from datetime import datetime
from . import utils
from rclpy.node import Node
from std_msgs.msg import Float32

I2C_ADDRESS = int(os.getenv('I2C_ADDRESS',100))
READ_CMD = os.getenv('READ_CMD','R')

class Ec(Node):

    def __init__(self):
        super().__init__('ec')
        self.publisher_ = self.create_publisher(Float32, 'ec', 10)
        timer_period = 3 # seconds
        self.device = utils.get_device(I2C_ADDRESS)
        if (self.device):
            self.get_logger().info('Starting ec')
            self.timer = self.create_timer(timer_period, self.timer_callback)

        else:
            self.get_logger().info('Unable to start ec. No device found. Tried address "%d"' % I2C_ADDRESS)
       
    def timer_callback(self):
        reading = Float32()
        response = self.device.query(READ_CMD)
        if not response:
            self.get_logger().info('No reponse from probe')
        
        else:
            reading = self.device.query(READ_CMD)
            reading = reading.strip()
            reading.data = float(reading)
            self.publisher_.publish(reading)
            self.get_logger().info('Publishing: %d' % reading.data)
        
def main(args=None):
    rclpy.init(args=args)
    ec = Ec()
    rclpy.spin(ec)

    ec.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()