import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    pkg_share = get_package_share_directory('simulasi-2025')
    default_model_path = os.path.join(pkg_share, 'description/diff_drive_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/navigation.rviz')
    world_path = os.path.join(pkg_share, 'worlds/my_world.sdf')

    # Node untuk robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    # Node untuk joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path]
    )
    
    # Node untuk rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
 
    # Node untuk spawn entity (robot) di Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'bot', 
                   '-topic', 'robot_description', 
                   '-x', '7.2', 
                   '-y', '-3.5',
                   '-z', '0.2',
                   '-Y', '-1.57'
                  ],
        output='screen'
    )

    # Node untuk menjalankan joy_node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    # Node untuk teleop_twist_keyboard
    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'  
    )
    # Node untuk menjalankan script autodrive
    autodrive_node = Node(
        package='simulasi-2025',  # Ganti dengan nama package Anda
        executable='autodrive.py',  # Menjalankan dengan python3
        name='autodrive_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'target_x': -7.2},
            {'target_y': 3.5},
        ]
)



    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        DeclareLaunchArgument('target_x', default_value='5.0', description='Koordinat X target'),
        DeclareLaunchArgument('target_y', default_value='5.0', description='Koordinat Y target'),

 
       
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),

       
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node,
        
        joy_node,
        teleop_twist_keyboard_node,
        autodrive_node
    ])
