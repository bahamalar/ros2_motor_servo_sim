from setuptools import find_packages, setup

package_name = 'motor_servo_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='bhkrky@gmail.com',
    description='Motor ve servo simulasyonu icin ROS2 paketi',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = motor_servo_sim.publisher_node:main',
            'subscriber_node = motor_servo_sim.subscriber_node:main',
        ],
    },
)
