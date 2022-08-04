from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    async_auto_argument = DeclareLaunchArgument('async_auto', default_value='False',
                                           description='Async auto heatmapping using HeatmappingPlanner')
    sync_auto_argument = DeclareLaunchArgument('sync_auto', default_value='False',
                                               description='Sync auto heatmapping using HeatmappingPlanner')

    async_auto = LaunchConfiguration('async_auto')
    sync_auto = LaunchConfiguration('sync_auto')

    floor_heat_mapper_node = Node(
        package='floor_heat_mapper',
        executable='floor_heat_mapper',
        name='floor_heat_mapper',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"sync": sync_auto},
            {"async": PythonExpression(['not ', sync_auto])}
        ]
    )

    heat_mapping_planner_node = Node(
        condition=IfCondition(PythonExpression([async_auto, ' or ', sync_auto])),
        package='floor_heat_mapper',
        executable='heat_mapping_planner.py',
        name='heat_mapping_planner',
        output='screen',
        emulate_tty=True,
        # parameters=[
        #     {"sync": sync_auto},
        # ]
    )

    return LaunchDescription([
        sync_auto_argument,
        async_auto_argument,
        floor_heat_mapper_node,
        heat_mapping_planner_node
    ])
