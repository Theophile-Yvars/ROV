from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Le Cerveau (Logique de contrôle et centralisation)
        Node(
            package='rov_brain',
            executable='brain_node',
            name='brain',
            output='screen'
        ),

        # 2. Le Bridge Caméra (Python)
        Node(
            package='rov_hardware',
            executable='camera_bridge_node.py', 
            name='camera_front',
            output='screen'
        ),

        # 3. Capteur de Température Interne TMP102 (C++)
        Node(
            package='rov_hardware',
            executable='temp_node',
            name='temp_internal',
            output='screen',
            parameters=[{'i2c_address': 0x48}]
        ),

        # 4. Capteur de Pression & Profondeur Externe MS5837 (C++) 
        Node(
            package='rov_hardware',
            executable='pression_node',
            name='pressure_external',
            output='screen'
        ),

        # 5. Le serveur vidéo (Stream HTTP pour le Dashboard - Port 8080)
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

        # 6. Température CPU du Raspberry Pi
        Node(
            package='rov_hardware',
            executable='cpu_temp_node',
            name='pi_cpu_temp',
            output='screen'
        ),

        # 7. Centrale Inertielle (IMU BNO055)
        Node(
            package='rov_hardware',
            executable='imu_node',
            name='imu_gyro',
            output='screen',
            parameters=[{'i2c_address': 0x29}]
        ),

        # 8. Rosbridge (Communication WebSocket pour le Joystick - Port 9090)
        Node(
            package='rosbridge_server', 
            executable='rosbridge_websocket', 
            name='rosbridge',
            output='screen'
        ),

        # 9. NOUVEAU : Télémétrie Batterie (ADC ADS1115)
        Node(
            package='rov_hardware',
            executable='battery_node.py',
            name='battery_monitor',
            output='screen',
            # Optionnel : si tu veux forcer l'adresse I2C via paramètre
            parameters=[{'i2c_address': 0x49}] 
        ),

        # 10. Contrôleur Servo Caméra
        Node(
            package='rov_hardware',
            executable='camera_servo_node', # Assure-toi que c'est le nom de l'exécutable dans CMakeLists.txt
            name='camera_servo',
            output='screen'
        ),

        # --- Contrôleur des Propulseurs / Moteurs (F2838 + ESCs) ---
        Node(
            package='rov_hardware',
            executable='thruster_controller_node',  # Doit correspondre exactement au nom dans CMakeLists.txt
            name='thruster_controller',
            output='screen'
        )
    ])