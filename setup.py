from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot4_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pleopardi',
    maintainer_email='paolo.leopardi@uni-konstanz.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot4_controller_node = turtlebot4_controller.turtlebot4_controller_node:main',
            'object_counter_node = turtlebot4_controller.object_counter_node:main',
            'cliff_sensor_node = turtlebot4_controller.cliff_sensor_node:main',
            'light_sensor_node = turtlebot4_controller.light_sensor_node:main'
        ],
    },
)
