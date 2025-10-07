from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    rqt_arg = DeclareLaunchArgument("rqt", 
                                    default_value="false",
                                    description="Run with rqt or not"
    )

    # sonar_arg = DeclareLaunchArgument("sonar", 
    #                                   default_value="true",
    #                                   description="Enable sonar nodes"
    # )

    model_arg = DeclareLaunchArgument("model", 
                                      default_value="M3000d",
                                      description="sonar model (M750d / M1200d / M300d)"
    )

    params_config = os.path.join(
        get_package_share_directory('sonar_oculus'),
        'config',
        'sonar_oculus_params.yaml'
        )
    
    # Node for sonar
    sonar_node = Node(
        package="sonar_oculus",
        executable="sonar_oculus",
        name=[ "sonar_oculus_", LaunchConfiguration("model") ],
        output="screen",
    #   condition=IfCondition(LaunchConfiguration("sonar")),
        parameters=[params_config], 
    )

    # Node for sonar viewer
    sonar_viewer_node = Node(
        package="sonar_oculus",
        executable="oculus_viewer.py",
        name=["sonar_oculus", LaunchConfiguration("model"), "_viewer"],
        output="screen",
        arguments=[LaunchConfiguration("model")],
    )
    
    return LaunchDescription([
        rqt_arg,
    #   sonar_arg,
        model_arg,
        sonar_node,
        sonar_viewer_node,
    ])