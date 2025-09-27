import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    uds_description_dir = get_package_share_directory("uds_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(uds_description_dir, "urdf", "body" ,"uds_robot.urdf.xacro"),
                                      description="Absolute path to robot urdf file")
    

    robot_description = ParameterValue(Command([
        "xacro ",
        LaunchConfiguration("model"),
        " is_sim:=True"
    ]),
    value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     }]
    )

    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     parameters=[{"robot_description": robot_description,
    #                 }]
    # )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{"robot_description": robot_description,
                    'use_sim_time' : False}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(uds_description_dir, "rviz", "display.rviz")]
    )


    return LaunchDescription([
        model_arg,
        # joint_state_publisher_node,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])