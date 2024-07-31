import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, String


class SwitchLightNode(Node):
    def __init__(self):
        super().__init__('switch_light')
        self.light_level_subscription = self.create_subscription(
            Int16,
            'lightness',
            self.light_level_callback,
            10)
        self.status_publisher = self.create_publisher(String, 'light_status', 10)
        self.light_threshold = 500  # Уровень освещенности для включения света


    def light_level_callback(self, msg):
        if msg.data is not None:
            if msg.data > self.light_threshold:
                self.get_logger().info("Turn off the light, it's bright enough!")
                status_msg = String() 
                status_msg.data = "Light is off"
                self.status_publisher.publish(status_msg)
            else: 
                self.get_logger().info("Turn on the light, it's too dark outside!") 
                status_msg = String()
                status_msg.data = "Light is on"
                self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    switch_light_node = SwitchLightNode()
    rclpy.spin(switch_light_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()