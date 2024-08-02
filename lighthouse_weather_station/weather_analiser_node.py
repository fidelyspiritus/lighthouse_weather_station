import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float64, String

class WeatherAnalyseNode(Node):
    def __init__(self):
        super().__init__('weather_analyse')

        self.topics = {
            'temperature': Int16,
            'humidity': Int16,
            'lightness': Int16,
            'wind_speed': Float64,
            'wind_direction': Int16  # оставим подписку, но исключим из анализа
        }

        self.values = {topic: None for topic in self.topics.keys()}
        self.subscribers = []
        self.publisher_ = self.create_publisher(String, 'weather_forecast', 10)

        # Подписка на все топики
        for topic, msg_type in self.topics.items():
            self.subscribers.append(
                self.create_subscription(msg_type, topic, self.create_callback(topic), 10)
            )

    def create_callback(self, topic):
        def callback(msg):
            self.values[topic] = msg.data
            self.analyse_weather()
        return callback

    def analyse_weather(self):
        forecasts = []
        parameters = []

        # Оценка условий и добавление параметров
        for topic, value in self.values.items():
            if value is not None:
                condition, parameter = self.get_condition_and_parameter(topic, value)
                if condition:
                    forecasts.append(condition)
                if parameter:
                    parameters.append(parameter)

        # Составление итогового прогноза
        forecast = "Weather conditions are: " + ", ".join(set(forecasts))
        parameters_str = ", ".join(parameters)
        full_forecast = f"{forecast}. Parameters: {parameters_str}."

        self.publish_forecast(full_forecast)

    def get_condition_and_parameter(self, topic, value):
        condition = None
        parameter = None

        match topic:
            case 'temperature':
                condition = self.get_temperature_condition(value)
                parameter = f"Temperature: {value}°C"
            case 'humidity':
                parameter = f"Humidity: {value}%"
                if self.values['temperature'] is not None and self.values['temperature'] < 0 and value > 70:
                    condition = "Snowy"
                elif value > 70:
                    condition = "Rainy"
            case 'lightness':
                condition = self.get_light_condition(value)
                parameter = f"Lightness: {value} Lux"
            case 'wind_speed':
                condition = self.get_wind_condition(value)
                parameter = f"Wind Speed: {value} m/s"

        return condition, parameter

    def get_light_condition(self, lightness):
        if lightness > 800:
            return "Sunny"
        elif 300 <= lightness <= 800:
            return "Cloudy"
        return None

    def get_temperature_condition(self, temperature):
        if temperature > 20:
            return "Warm"
        elif 10 <= temperature <= 20:
            return "Mild"
        elif temperature < 0:
            return "Cold"
        return None

    def get_wind_condition(self, wind_speed):
        if 10 <= wind_speed <= 20:
            return "Windy"
        elif wind_speed > 20:
            return "Stormy"
        return None

    def publish_forecast(self, forecast):
        msg = String()
        msg.data = forecast
        self.publisher_.publish(msg)
        self.get_logger().info(f"Weather forecast: {forecast}")

def main(args=None):
    rclpy.init(args=args)
    weather_analyse = WeatherAnalyseNode()
    rclpy.spin(weather_analyse)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
