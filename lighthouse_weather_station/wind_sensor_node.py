import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int16
from random import randint, uniform


class WindSensorNode(Node):
    def __init__(self):
        super().__init__('wind_sensor')
        self.speed_publisher = self.create_publisher(Float64, 'wind_speed', 10)
        self.direction_publisher = self.create_publisher(Int16, 'wind_direction', 10)
        self.timer_period = 1.0  # Период в секундах
        self.timer = self.create_timer(self.timer_period, self.publish_wind_data)


    def publish_wind_data(self):
        wind_speed = round(uniform(0.0, 20.0), 2)  # Случайная скорость ветра от 0 до 20 м/с
        wind_direction = randint(0, 360)  # Случайное направление ветра в градусах

        speed_msg = Float64()
        speed_msg.data = wind_speed
        self.speed_publisher.publish(speed_msg)

        direction_msg = Int16()
        direction_msg.data = wind_direction
        self.direction_publisher.publish(direction_msg)

        self.get_logger().info(f'Published Wind Speed: {wind_speed} m/s, Direction: {wind_direction} degrees')


def main(args=None):
    rclpy.init(args=args)
    wind_sensor_node = WindSensorNode()
    rclpy.spin(wind_sensor_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()