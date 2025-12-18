# Inicializar el sistema de teleoperación y transmisión desde el robot
ros2 launch robot_canbus teleop_can.launch.py

# Ejecutar el receptor de streaming de video en la estación local
ros2 run video_streamer video_rx

# Habilitar el dashboard de supervisión y telemetría
ros2 run gui_dashboard dashboard_node
