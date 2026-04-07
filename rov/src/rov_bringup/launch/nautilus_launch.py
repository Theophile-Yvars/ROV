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

        # 2. Le Bridge Caméra
        # Note : On utilise le nom de l'exécutable défini dans CMakeLists.txt
        Node(
            package='rov_hardware',
            executable='camera_bridge_node.py', 
            name='camera_front',
            output='screen'
        ),

        # 3. Capteur de Température TMP102 (C++)
        # Nouveau Node ajouté ici !
        Node(
            package='rov_hardware',
            executable='temp_node',
            name='temp_sensor',
            output='screen'
        ),

        # 4. Le serveur vidéo (Stream HTTP pour le Dashboard - Port 8080)
        Node(
            package='web_video_server', 
            executable='web_video_server', 
            name='web_video',
            parameters=[{
                'port': 8080,
                'default_transport': 'raw',
                'quality': 100 
            }]
        ),

        # 5. Rosbridge (Communication WebSocket pour le Joystick - Port 9090)
        Node(
            package='rosbridge_server', 
            executable='rosbridge_websocket', 
            name='rosbridge',
            output='screen'
        )
    ])