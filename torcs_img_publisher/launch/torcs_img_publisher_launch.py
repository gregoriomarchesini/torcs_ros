import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_dir = get_package_share_directory('torcs_img_publisher')
    robot_id = LaunchConfiguration('robot_id')
    use_namespace = LaunchConfiguration('use_namespace')

    ld = LaunchDescription([

        DeclareLaunchArgument('use_namespace', description='Execute all nodes/topics within /robot_id namespace', default_value='False'),
        DeclareLaunchArgument('robot_id', description='Robot ID', default_value='leo_sim'),

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=['/', robot_id]
            ),

            Node(
                package = 'torcs_img_publisher',
                name = 'torcs_img_publisher',
                executable = 'torcs_img_publisher_node',
                parameters = [os.path.join(package_dir, 'config', 'params.yaml'),],
                remappings = [
                    ('ego_view', 'pov_image'),
                ],
            ),
        ]),
    ])
return ld
