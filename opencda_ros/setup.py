"""
Setup for opencda_ros
"""
import os
from glob import glob
ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(
        packages=['opencda_ros'],
        package_dir={'': 'src'}
    )

    setup(**d)

elif ROS_VERSION == 2:
    from setuptools import setup

    package_name = 'opencda_ros'
    setup(
        name=package_name,
        version='0.0.0',
        packages=['src/' + package_name],
        data_files=[
            ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + '/config',
             ['config/objects.json']),
            (os.path.join('share', package_name), glob('launch/*.launch.py'))
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='CARLA Simulator Team',
        maintainer_email='carla.simulator@gmail.com',
        description='CARLA spawn_objects for ROS2 bridge',
        license='MIT',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'opencda_ros = src.opencda_ros.carla_spawn_objects:main',
                'set_initial_pose = src.opencda_ros.set_initial_pose:main',
                'vehicle_data_subscriber = src.opencda_ros.vehicle_data_subscriber:main',
                'Suntrax_platoon_main = src.opencda_ros.Suntrax_platoon_main:main'
            ],
        },
    )
