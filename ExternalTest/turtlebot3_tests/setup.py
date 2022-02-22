from setuptools import setup
from glob import glob

package_name = 'turtlebot3_tests'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/maps', glob('maps/*.pgm')),
        (f'share/{package_name}/maps', glob('maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duc',
    maintainer_email='ducanh.than@rapyuta-robotics.com',
    description='ROS2 tests for turtlebot3',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
