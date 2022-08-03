from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    manual_argument = DeclareLaunchArgument('manual', default_value='false',
                                               description='Async manual heatmapping using teleop_twist')
    semi_auto_argument = DeclareLaunchArgument('semi_auto', default_value='false',
                                               description='Async semi-auto heatmapping using nav2')
    async_auto_argument = DeclareLaunchArgument('async_auto', default_value='false',
                                           description='Async auto heatmapping using HeatmappingPlanner')
    sync_auto_argument = DeclareLaunchArgument('sync_auto', default_value='false',
                                               description='Sync auto heatmapping using HeatmappingPlanner')


    floor_heat_mapper_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    return LaunchDescription([

    ])
