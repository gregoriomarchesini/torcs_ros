import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    package_dir = get_package_share_directory('torcs_ros_bringup')
    torcs_ros_client_dir = get_package_share_directory('torcs_ros_client')
    torcs_ros_drive_ctrl_dir = get_package_share_directory('torcs_ros_drive_ctrl')
    torcs_img_publisher_dir = get_package_share_directory('torcs_img_publisher')

    robot_id = LaunchConfiguration('robot_id')
    use_namespace = LaunchConfiguration('use_namespace')
    use_rviz = LaunchConfiguration('use_rviz')
    use_driver = LaunchConfiguration('use_driver')
    publish_game_img = LaunchConfiguration('publish_game_img')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    torcs_ros_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([torcs_ros_client_dir, 'launch', 'torcs_ros_client_launch.py'])]),
            launch_arguments={
                'robot_id':robot_id,
                'use_namespace': use_namespace,
            }.items()
    )

    torcs_img_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([torcs_img_publisher_dir, 'launch', 'torcs_img_publisher_launch.py'])]),
            condition=IfCondition(publish_game_img),
            launch_arguments={
                'robot_id':robot_id,
                'use_namespace': use_namespace,
            }.items()
    )

    torcs_ros_drive_ctrl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([torcs_ros_drive_ctrl_dir, 'launch', 'torcs_ros_drive_ctrl_launch.py'])]),
            condition=IfCondition(use_driver),
            launch_arguments={
                'robot_id':robot_id,
                'use_namespace': use_namespace,
            }.items()
    )

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                ],
    )

    ld = LaunchDescription([

        DeclareLaunchArgument('use_namespace', description='Execute all nodes/topics within /robot_id namespace', default_value='False'),
        DeclareLaunchArgument('robot_id', description='Robot ID', default_value='leo_sim'),
        DeclareLaunchArgument('use_rviz', default_value='True', description='Whether to start rviz'),
        DeclareLaunchArgument('use_driver', default_value='True', description='Whether to start driving'),
        DeclareLaunchArgument('publish_game_img', default_value='True', description='Whether to start the Game image'),
        DeclareLaunchArgument('rviz_config_file', default_value=os.path.join(package_dir, 'config', 'torcs.rviz'), description='Full path to the RVIZ config file to use'),
    ])

    ld.add_action(torcs_ros_client)
    ld.add_action(torcs_img_publisher)
    ld.add_action(torcs_ros_drive_ctrl)
    ld.add_action(rviz_cmd)

    return ld
