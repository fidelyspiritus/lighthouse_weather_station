from setuptools import setup
import os

package_name = 'lighthouse_weather_station'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Включаем launch файлы
        (os.path.join('share', package_name, 'launch'), ['launch/weather_station.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anastasiia Markova',
    maintainer_email='anastasiia.a.markova@gmail.com',
    description='Imitation of a weather station on a lighthouse',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'temperature_sensor = lighthouse_weather_station.temperature_sensor_node:main',
        'light_sensor = lighthouse_weather_station.light_sensor_node:main',
        'display = lighthouse_weather_station.display_node:main',
        'switch_light = lighthouse_weather_station.switch_light_node:main',
        'wind_sensor = lighthouse_weather_station.wind_sensor_node:main',
        'weather_analise = lighthouse_weather_station.weather_analiser_node:main'
        ],
    },
)
