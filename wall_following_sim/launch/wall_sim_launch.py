from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

config_dir = get_package_share_directory("wall_following_sim")

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_following_sim',
            executable='vehicle_simulator',
            name='vehicle_simulator',
            output='screen',
            parameters = [
                {'starting_position': [0.7487, 1.35, 0.0]},  #  m in UTM coord sys
                {'starting_ang_deg': 22.0},
                {'L_wheelbase_m': 0.33}  # dist between axles in meters
            ] 
        ),
        Node(
            package='wall_following_sim',
            executable='calc_steer',
            name='calc_steer',
            output='screen',
            parameters = [
                {'theta_deg': 42.0},  # angle from perp beam to beam 2, deg
                {'d_LookAhead': 2.5}, # distance to look-ahead point, m
                {'d_desired': 1.0},  # desired distance from wall, m
                {'speed': 1.0}, # m/sec
                {'Kp': 0.5},
                {'Ki': 0.1},
                {'Kd': 0.1},
                {'starting_delay': 5.0}, # seconds
                {'save_to_file': True},
                {'file_name': 'my_outfile.csv'},
            ] 
        ),
        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=["-d", config_dir + "/rviz/rviz_wall_following_sim.rviz"]
        )
    ])