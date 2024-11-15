# motor_control_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the node as a variable
    motor_control_node = Node(
        package='ethercat_servo_control',
        executable='motor_control_node',  
        name='motor_control_node',
        output='screen'
    )

    # Return the LaunchDescription with the node variable
    return LaunchDescription([
        motor_control_node
    ])
