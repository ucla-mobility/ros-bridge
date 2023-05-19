import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([

        # ------------------------ parameters --------------------------
        # 1. server config
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        # 2. world config
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='10'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='/home/xuhan/carla-ros-bridge/src/ros-bridge/opencda_ros/launch/suntrax_oval_map.xodr',
            description='Either use an available CARLA town (eg. "Town01") or an OpenDRIVE file (ending in .xodr)'
        ),
        # 3. simulation config
        launch.actions.DeclareLaunchArgument(
            name='passive',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.005'
        ),
        # 4. vehicle and senor config 
        launch.actions.DeclareLaunchArgument(
            name='register_all_sensors',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ego_vehicle_role_name',
            default_value=["mainline_ADS_vehicle_1", "mainline_ADS_vehicle_2", "single_ADS_vehicle",\
                           "ego_vehicle","hero0", "hero1", "hero2","hero3", "hero4", "hero5", \
                           "hero6", "hero7", "hero8", "hero9"],
            description='Role names to identify ego vehicles. '
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_sensors_only',
            default_value='False'
        ),
        # 4.a vehicle one 
        launch.actions.DeclareLaunchArgument(
            name='ADS_vehicle_spawn_file',
            default_value=get_package_share_directory(
                'opencda_ros') + '/config/objects.json',
            description='Configuration files for all spawned ego vehicles. '
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point_ADS_vehicles',
            default_value='None',
            description='CARLA will report error signal 11 if specify a \
                        location. Try to modify source code and spawn vehicle\
                        on specific location based on scenario and role name.'
        ),
        

        # ---------------------- launch nodes --------------------------
        # 1. ROS bridge 
        launch_ros.actions.Node(
            package='carla_ros_bridge',
            executable='bridge',
            name='carla_ros_bridge',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': True
                },
                {
                    'host': launch.substitutions.LaunchConfiguration('host')
                },
                {
                    'port': launch.substitutions.LaunchConfiguration('port')
                },
                {
                    'timeout': launch.substitutions.LaunchConfiguration('timeout')
                },
                {
                    'passive': launch.substitutions.LaunchConfiguration('passive')
                },
                {
                    'synchronous_mode': launch.substitutions.LaunchConfiguration('synchronous_mode')
                },
                {
                    'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command')
                },
                {
                    'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
                },
                {
                    'town': launch.substitutions.LaunchConfiguration('town')
                },
                {
                    'register_all_sensors': launch.substitutions.LaunchConfiguration('register_all_sensors')
                },
                {
                    'ego_vehicle_role_name': launch.substitutions.LaunchConfiguration('ego_vehicle_role_name')
                }
            ]
        ),

        # 2. Spawn objects (vehicle and sensors)
        launch_ros.actions.Node(
            package='carla_spawn_objects',
            executable='carla_spawn_objects',
            name='carla_spawn_objects',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'objects_definition_file': launch.substitutions.LaunchConfiguration('ADS_vehicle_spawn_file')
                },
                {
                    'spawn_point_ADS_vehicles': launch.substitutions.LaunchConfiguration('spawn_point_ADS_vehicles')
                },
                {
                    'spawn_sensors_only': launch.substitutions.LaunchConfiguration('spawn_sensors_only')
                }
            ]
        ),

        # todo: 
        # 3. wait for command (no need for now)
            # modify "carla manual control"
        # 4. Read all vehicle data for OpenCDA 
            # right code to subscribe all data for opencda
            # openCDA class access dataloader to access information
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
