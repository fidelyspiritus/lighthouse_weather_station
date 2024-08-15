from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lighthouse_weather_station',
            executable='temperature_sensor',
            name='temperature_sensor',
            output='screen'
        ),
        Node(
            package='lighthouse_weather_station',
            executable='light_sensor',
            name='light_sensor',
            output='screen'
        ),
        Node(
            package='lighthouse_weather_station',
            executable='wind_sensor',
            name='wind_sensor',
            output='screen'
        ),
        Node(
            package='lighthouse_weather_station',
            executable='switch_light',
            name='switch_light',
            output='screen'
        ),
        Node(
            package='lighthouse_weather_station',
            executable='weather_analise',
            name='weather_analise',
            output='screen'
        ),
        Node(
            package='lighthouse_weather_station',
            executable='display',
            name='display',
            output='screen'
        )
    ])