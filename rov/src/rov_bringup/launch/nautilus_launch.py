from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Le Cerveau (Lecture Gyro + Moteurs)
        Node(package='robot_brain', executable='brain_node', name='brain'),

        # 2. La Caméra (Qualité HD pour ta Cam v3)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera_front',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [1280, 720], # 720p est le "sweet spot" pour le streaming
                'pixel_format': 'MJPG',    # Crucial pour la vitesse sur Raspberry
                'output_encoding': 'rgb8',
                'pauze_mode': False,
                'brightness': 50,          # Ajustable selon la clarté de l'eau
            }]
        ),

        # 3. Le serveur vidéo (Flux HTTP - Port 8080)
        Node(
            package='web_video_server', 
            executable='web_video_server', 
            name='web_video'
        ),

        # 4. Rosbridge (Données JSON - Port 9090)
        Node(
            package='rosbridge_server', 
            executable='rosbridge_websocket', 
            name='rosbridge'
        )
    ])