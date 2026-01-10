from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    node =  Node(
            package="moveit_control_cpp",
            executable="moveit_pose",
            output='screen',
            # prefix="valgrind --tool=memcheck --leak-check=full --track-origins=yes --show-reachable=yes --log-file=/home/slz/memory/valgrind-%p.log",
            prefix="valgrind --tool=memcheck --trace-children=yes --leak-check=full --show-leak-kinds=all --track-origins=yes --num-callers=40 --time-stamp=yes --error-limit=no --read-var-info=yes",
            arguments=['--ros-args', '--log-level', 'INFO']
        )
    
    return LaunchDescription([
       node
    ])