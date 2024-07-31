import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float64, String

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')

        # Определение топиков, соответствующих типов сообщений и единиц измерения
        self.topics = {
            'temperature': (Int16, 'Celsius'),
            'humidity': (Int16, '%'), 
            'lightness': (Int16, 'Lux'),
            'light_status': (String, ''),
            'wind_speed': (Float64, 'm/s'),
            'wind_direction': (Int16, 'degrees')
        }

        # Список для хранения подписок
        self.subscribers = []

        # Подписка на все топики
        for topic, (msg_type, unit) in self.topics.items():
            self.subscribers.append(
                self.create_subscription(msg_type, topic, self.create_callback(topic, unit), 10))

    def create_callback(self, topic, unit):
        def callback(msg):
            # логирование данных в зависимости от типа сообщения и единиц измерения
            if isinstance(msg, Float64):
                self.get_logger().info(f'{topic.capitalize().replace("_", " ")}: {msg.data:.2f} {unit}')
            else:
                self.get_logger().info(f'{topic.capitalize().replace("_", " ")}: {msg.data} {unit}')
        return callback

def main(args=None):
    rclpy.init(args=args)
    display_node = DisplayNode()
    rclpy.spin(display_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
