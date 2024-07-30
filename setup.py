from setuptools import setup

package_name = 'lighthouse_weather_station'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'display = lighthouse_weather_station.display_node:main'
        ],
    },
)
