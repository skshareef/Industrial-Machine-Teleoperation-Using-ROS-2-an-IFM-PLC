from setuptools import setup
import os
from glob import glob

package_name = 'trench_roller_autonomy'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Copy the Python scripts into the install space
        (os.path.join('share', package_name), glob('launch/*.launch.py') if os.path.exists('launch') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@domain.tld',
    description='Trench roller autonomy nodes for controlling an IFM controller over TCP.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_teleop_node = trench_roller_autonomy.joy_teleop_node:main',
            'tcp_handler_node = trench_roller_autonomy.tcp_handler_node:main',
        ],
    },
)
