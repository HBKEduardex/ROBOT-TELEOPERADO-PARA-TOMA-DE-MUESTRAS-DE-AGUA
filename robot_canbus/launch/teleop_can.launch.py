from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1) JOY → lee el mando PS4 por Bluetooth
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {"deadzone": 0.05},
                {"autorepeat_rate": 20.0},
            ],
        ),

        # 2) TELEOP PS4 → publica /Data_PS4
        Node(
            package='teleop_master',
            executable='teleop_ps4',
            name='teleop_ps4_node',
            output='screen',
        ),

        # 3) CAN PS4 BRIDGE → mueve motores + UART + stream_cmd
        Node(
            package='robot_canbus',
            executable='can_ps4_bridge',
            name='can_ps4_bridge',
            output='screen',
        ),

        # 4) VIDEO TX → streaming de la cámara
        Node(
            package='robot_video_tx',
            executable='video_tx',
            name='video_tx',
            output='screen',
            # parameters=[ ... ]  # aquí si luego quieres pasarle resolución, fps, etc.
        ),
        # 🔵 NUEVO: logger de misión
        Node(
            package='robot_canbus',
            executable='mission_logger',
            name='mission_logger',
            output='screen'
        ),
        Node(
            package='robot_status',
            executable='status_node',
            name='robot_status',
            output='screen'
        ),
    ])
