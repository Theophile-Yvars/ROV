from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Le Cerveau (Logique de contrôle)
        Node(
            package='rov_brain',
            executable='brain_node',
            name='brain',
            output='screen'
        ),

        # 2. Le Bridge Caméra (Indispensable pour avoir l'image !)
        # Il lit /dev/video10 et publie sur le topic /image_raw
        Node(
            package='rov_hardware',
            executable='camera_bridge.py',
            name='camera_front',
            output='screen'
        ),

        # 3. Le serveur vidéo (Stream HTTP pour le Dashboard - Port 8080)
        Node(
            package='web_video_server', 
            executable='web_video_server', 
            name='web_video',
            parameters=[{
                'port': 8080,
                'default_transport': 'raw',
                'quality': 100 # Qualité max
            }]
        ),

        # 4. Rosbridge (Communication WebSocket pour le Joystick - Port 9090)
        Node(
            package='rosbridge_server', 
            executable='rosbridge_websocket', 
            name='rosbridge',
            output='screen'
        )
    ])