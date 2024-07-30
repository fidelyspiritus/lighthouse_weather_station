import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, String


class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        self.temperature_subscription = self.create_subscription(
            Int16,
            'temperature',
            self.temperature_callback,
            10)
        self.light_subscription = self.create_subscription(
            Int16,
            'lightness',
            self.light_callback,
            10)
        self.status_subscription = self.create_subscription(
            String,
            'light_status',
            self.status_callback,
            10)
        self.temperature_subscription  # предотвращение предупреждения о неиспользуемой переменной
        self.light_subscription
        self.status_subscription


    def temperature_callback(self, msg):
        self.get_logger().info('Temperature: "%s" Celsius' % msg.data)


    def light_callback(self, msg):
        self.get_logger().info('Light level: "%s" Lux' % msg.data)
   
    def status_callback(self, msg):
        self.get_logger().info('Light status: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    display_node = DisplayNode()
    rclpy.spin(display_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()