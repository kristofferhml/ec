import rclpy
import os
from datetime import datetime
from collections import deque
from . import utils
from rclpy.node import Node
from std_msgs.msg import Float32

I2C_ADDRESS = int(os.getenv('I2C_ADDRESS',100))
METRIC_INTERVAL = int(os.getenv('METRIC_INTERVAL',10))
READ_CMD = os.getenv('READ_CMD','R')
NODE_NAME = os.getenv('NODE_NAME','metric')
WINDOW_SIZE = int(os.getenv('WINDOW_SIZE',5))

class MetricNode(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        self.publisher_ = self.create_publisher(Float32, NODE_NAME, 10)
        timer_period = METRIC_INTERVAL # seconds
        self.device = utils.get_device(I2C_ADDRESS)
        self.measurements = deque(maxlen=WINDOW_SIZE)

        if (self.device):
            self.get_logger().info('Starting float metric node')
            self.timer = self.create_timer(timer_period, self.timer_callback)

        else:
            self.get_logger().info('Unable to start ec. No device found. Tried address "%d"' % I2C_ADDRESS)
       
    def timer_callback(self):
        reading = Float32()
        response = self.device.query(READ_CMD)
        if not response:
            self.get_logger().info('No reponse from probe')
        
        else:
            from_device = self.device.query(READ_CMD)
            from_device = from_device.strip('\x00')
            self.measurements.append(float(from_device))
            moving_average = sum(self.measurements) / len(self.measurements)

            reading.data = moving_average
            self.publisher_.publish(reading)
            self.get_logger().info(f"Current sensor data: {list(self.measurements)}")
            self.get_logger().info('Publishing: %.2f' % reading.data)
        
def main(args=None):
    rclpy.init(args=args)
    mn = MetricNode()
    rclpy.spin(mn)

    mn.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()