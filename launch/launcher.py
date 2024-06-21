import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    xml_file_name = "model/xml/model_1.xml"
    #xml_file_name = "model/bitcraze_crazyflie_2/scene.xml"
    xml_file = os.path.join(get_package_share_path("drone_mujoco"), xml_file_name)
    print(xml_file)
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=xml_file,
            description='Path to the model XML file.'
        ),
        Node(
            package='drone_mujoco',
            executable='my_node',
            name='simulate_node',
            output='screen',
            arguments=[LaunchConfiguration('model_path')],
        )
    ])