import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('otonom_surus')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # Senaryo seçimi: config/scenarios.yaml içinden dünya, başlangıç ve hedef
    senaryo_no = LaunchConfiguration('senaryo').perform(context)
    with open(os.path.join(pkg_dir, 'config', 'scenarios.yaml'), 'r') as f:
        senaryolar = yaml.safe_load(f)['senaryolar']
    if senaryo_no not in senaryolar:
        raise RuntimeError(
            f"Geçersiz senaryo '{senaryo_no}'. Seçenekler: {', '.join(senaryolar.keys())}")
    senaryo = senaryolar[senaryo_no]
    baslangic = senaryo['baslangic']
    hedef = senaryo['hedef']
    world_file = os.path.join(pkg_dir, 'worlds', senaryo['dunya'])

    # Model ve Gazebo ortam değişkenleri
    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    # Paketin kendi modelleri (arena_walls, goal_marker) model:// ile bulunabilsin
    model_paths = [
        os.path.join(pkg_dir, 'models'),
        '/usr/share/gazebo-11/models',
        os.path.join(tb3_gazebo_dir, 'models'),
    ]
    set_gazebo_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', ':'.join(model_paths))

    # 1. Gazebo Simülasyonu
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 2. Robot State Publisher
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_waffle.urdf'
    )
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    robot_description_content = robot_description_content.replace('${namespace}', '')

    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content
        }]
    )

    # 3. Robotu senaryonun başlangıç pozunda spawn etme (Geliştirilmiş 8m LiDAR'lı model)
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-file', os.path.join(pkg_dir, 'models', 'turtlebot3_waffle', 'model.sdf'),
            '-entity', 'waffle',
            '-x', str(baslangic['x']), '-y', str(baslangic['y']), '-z', '0.01',
            '-Y', str(baslangic['yaw'])
        ],
        output='screen'
    )

    # 3b. Hedef işaretini (sadece görsel) hedef konumuna yerleştirme
    spawn_goal = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-file', os.path.join(pkg_dir, 'models', 'goal_marker', 'model.sdf'),
            '-entity', 'hedef',
            '-x', str(hedef['x']), '-y', str(hedef['y']), '-z', '0.0'
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot, spawn_goal]
    )

    # 4. SLAM Toolbox (Kaymayı ve çift duvarı önleyen hassas parametreler)
    start_slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'transform_publish_period': 0.02,
                'map_update_interval': 0.5,
                'resolution': 0.05,
                'max_laser_range': 8.0,
                'minimum_travel_distance': 0.15,
                'minimum_travel_heading': 0.20,
                'transform_tolerance': 0.3,
                'queue_size': 20,
                'do_loop_closing': True,
                'correlation_search_space_dimension': 0.5,
                'correlation_search_space_resolution': 0.01,
                'loop_search_maximum_distance': 5.0,
            }
        ]
    )

    # 5. Nav2 Navigasyon Hattı
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'autostart': 'True',
        }.items()
    )

    # 6. Klavye Kontrol Terminali (İsteğe bağlı: teleop:=true)
    start_teleop = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('teleop')),
        cmd=['gnome-terminal', '--', 'bash', '-c', 'source ~/Masaüstü/ros2_deneme/install/setup.bash && ros2 run otonom_surus teleop; exec bash'],
        output='screen'
    )

    # 7. RViz2 (Hazır Nav2 konfigürasyonu ile başlatma)
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'otonom_surus.rviz')
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # 8. Otomatik Görev Başlatıcı (İsteğe bağlı: otonom:=true)
    start_autonomous = TimerAction(
        period=12.0,
        actions=[
            Node(
                condition=IfCondition(LaunchConfiguration('otonom')),
                package='otonom_surus',
                executable='navigator',
                name='otonom_navigator',
                output='screen',
                parameters=[{
                    'senaryo': senaryo_no,
                    'use_sim_time': True
                }]
            )
        ]
    )

    return [
        set_model,
        set_gazebo_model_path,
        start_gazebo,
        start_robot_state_publisher,
        delayed_spawn,
        start_slam,
        start_nav2,
        start_teleop,
        start_rviz,
        start_autonomous
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'senaryo', default_value='1',
            description='Test senaryosu: 1 (kolay), 2 (yogun), 3 (ev planı)'
        ),
        DeclareLaunchArgument(
            'teleop', default_value='false',
            description='Klavye kontrol terminalini aç (true/false)'
        ),
        DeclareLaunchArgument(
            'otonom', default_value='false',
            description='Nav2 başladıktan sonra hedefe otomatik sürüş başlat (true/false)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
