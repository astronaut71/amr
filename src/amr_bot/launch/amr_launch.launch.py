from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='amr_bot').find('amr_bot')
    #slam_params_file_path = os.path.join(get_package_share_directory('your_package_name'), 'config', 'mapper_params_online_async.yaml')


# Static Transform Publishers
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.02', '0', '-0.02', '0', '0', '0', 'base_link', 'laser_link'],
    )

    static_tf_base_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.002', '0', '-0.002', '0', '0', '0', 'base_link', 'odom'],
    )

    static_tf_base_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    camera2base_node = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = ['-0.04','0', '0', '0', '0', '0', 'base_link', 'camera_link'],
    )

    #ros2 run realsense2_camera realsense2_camera_node
    #rs_camera_node = Node(
        #package = 'realsense2_camera',
        #executable = 'realsense2_camera_node',
        #using unite_imu_method is important, or the imu messages will be separated
        #arguments = {'align_depth': True, 'linear_accel_cov': '1.0', 'unite_imu_method':'2' ,
        #'initial_reset':'true', 'enable_gyro':'true', 'enable_accel':'true'},
    #)

    #include_realsense2_camera = IncludeLaunchDescription(
        #PythonLaunchDescriptionSource(os.path.join(
        #get_package_share_directory('realsense2_camera'),
        #'launch',
        #'rs_launch.py' )),
        #launch_arguments={
            #'align_depth.enable': 'true',
            #'linear_accel_cov': '1.0',
            #'unite_imu_method': '2', 'initial_reset':'true', 'enable_gyro':'true', 'enable_accel':'true'

            # 'filters': 'pointcloud',
        #}.items()
    #)

    include_realsense2_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
        )),
        launch_arguments={
            'align_depth.enable': 'true',
            'linear_accel_cov': '1.0',
            'unite_imu_method': '2',
            'initial_reset':'true',
            'enable_gyro':'true',
            'enable_accel':'true',
            'enable_infra1':'true',
            'enable_infra2':'true',
            'gyro_fps':'200',
            'accel_fps': '250',
            'enable_sync': 'true'
        # 'filters': 'pointcloud',
        }.items()
    )


    # Lidar Driver Launch
    lslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('lslidar_driver'), 'launch', 'lslidar_launch.py')),
    )

    # Corrected Roboteq Driver Launch
    roboteq_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('roboteq_ros2_driver'), 'launch', 'roboteq_ros2_driver.launch.py') ),
        #launch_arguments={'config': os.path.join(get_package_share_directory('roboteq_ros2_driver'), 'config', 'roboteq.yaml')}.items(),
    )

    # filter that fuses imu msg into odom msg
        #download from here ->https://github.com/CCNYRoboticsLab/imu_tools/tree/humble
    imu_to_odom_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{
        'use_mag': False,
        'mag_bias_x': 0.0,
        'mag_bias_y': 0.0,
        'mag_bias_z': 0.0,
        'orientation_stddev': 0.02,
        'gain': 0.02,
        'zeta': 0.002,
        'publish_tf': False
        }],
        remappings=[
        #change to your camera imu topic
        ('/imu/data_raw',  '/camera/camera/imu'), ],
    )

    #launch frame matcher
    points_xyzrgb_node = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        #namespace='/rtabmap',
        output='screen',
        parameters=[{'decimation': 4, 'voxel_size': 0.05, 'approx_sync': False}],
        remappings=[('cloud', '/camera/camera/depth/image_rect_raw')],
    )

    #rgbd_odometry_node = Node(
        #package='rtabmap_odom',
        #executable='rgbd_odometry',
        #output='screen',
        #parameters=[{
        #'frame_id': 'camera_link',
        #'subscribe_rgbd':False,
        #'publish_tf': False,
        #'publish_null_when_lost': False,
        #'guess_from_tf': True,
        #'Odom/FillInfoData': True,
        #'Odom/ResetCountdown': 1,
        #'Vis/FeatureType': 6,
        #'OdomF2M/MaxSize': 1000,}],
        #remappings=[
        #('rgb/image', '/camera/camera/color/image_raw'),
        #('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
        #('rgb/camera_info', '/camera/camera/color/camera_info'),
        #('odom', 'odom/rgbd1'),],
    #)

    rgbd_odom_node = Node(
        package='rtabmap_odom', executable='stereo_odometry', output='screen',
        parameters=[{
        'frame_id':'camera_link',
        'subscribe_depth':True,
        #'subscribe_stereo':True,
        'subscribe_odom_info':True,
        'approx_sync':False,
        'wait_imu_to_init':True}],
        remappings=[
        ('imu', '/imu/data'),
        #('rgb/image', '/camera/camera/infra1/image_rect_raw'),
        ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
        #('rgb/camera_info', '/camera/camera/infra1/camera_info'),
        ('left/camera_info', '/camera/camera/infra1/camera_info'),
        #('depth/image', '/camera/camera/depth/image_rect_raw'),
        ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
        ('right/camera_info', '/camera/camera/infra2/camera_info'),
        ('odom', 'odom/rgbd')],
        
    )

    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=[{
        'frame_id': 'camera_link',
        'publish_tf': False,
        'approx_sync':True,
        'publish_null_when_lost': True,
        'Odom/FillInfoData': True,
        'Odom/ResetCountdown': 5,}],
        #change to your camera topics
        remappings=[
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('odom', 'odom/rgbd'),],
    )


    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[{
        'frame_id': 'laser_link',
        'subscribe_depth':True,
        'subscribe_scan_cloud': False, 
        'scan_cloud_max_points': 1000,
        'scan_range_max': 20.0,
        'scan_range_min': 0.02,
        'subscribe_scan':True,         
        'subscribe_rgbd':False,
        'subscribe_rgb':False,
        'use_sim_time': False,
        'odom_frame_id':'odom',
        #'queue_size': 10,
        'queue_size': 10,
        'map_always_update': True,
        'map_empty_ray_tracing': True,
        'approx_sync': True,
        'publish_tf': False,}],
        remappings=[
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/depth/image_raw' ),
        ('odom', 'odom/icp'),],
    )
    
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        #parameters=[os.path.join(ThisLaunchFileDir(), 'config/ekf.yaml')],
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')],

    )

    # gmapping Launch
    gmapping_node = Node(
        package ='slam_gmapping',
        executable ='slam_gmapping',
        output='screen',
        parameters=[{'use_sim_time':False,'base_frame':'base_link','map_frame':'map','odom_frame':'odom',}],
    )

    slam_toolbox_launch_file = os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    slam_params_file_path = os.path.join(get_package_share_directory('amr_bot'), 'config', 'mapper_params_online_async.yaml')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'slam_toolbox_config_file': slam_params_file_path}],
        remappings=[
        ('/scan', '/scan'), # Remap from /scan to your custom scan topic
        # Add more remappings as needed
        ],
    )   

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        launch_arguments={
        'autostart': 'true',
        #'use_lifecycle_manager': ,
        'use_sim_time': 'false',
        'slam_params_file': slam_params_file_path}.items(),
    )

    #nav2_launch_file = os.path.join(get_package_share_directory('navigation2'), 'launch', 'navigation_launch.py')
    nav2_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    nav2_params_file_path = os.path.join(get_package_share_directory('amr_bot'), 'config', 'nav2_params.yaml')

    nav2_node = Node(
        #package='nav2_bringup',
        package='nav2_planner',
        #package='nav2_controller',
        #executable='bringup',
        executable='planner_server',
        #executable='controller_server',
        #name='nav2_bringup',
        name='planner_server',
        output='screen',
        #respawn= use_respawn,
        respawn_delay=2.0,
        parameters=[{'configured_params': nav2_params_file_path}],
        remappings=[
        ('/scan', '/scan'), ('/cmd_vel', 'cmd_vel_nav'),# Remap from /scan to your custom scan topic
        # Add more remappings as needed
        ],
    )   

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
        'autostart': 'true',
        #'use_lifecycle_manager': ,
        'use_sim_time': 'False',
        'params_file': nav2_params_file_path}.items(),
    )

    return LaunchDescription([
            static_tf_base_to_laser,
            static_tf_base_to_odom,
            static_tf_base_to_footprint,
            lslidar_launch,
            roboteq_driver_launch,
            imu_to_odom_node,
            #rgbd_odometry_node,
            #rgbd_odom_node,
            robot_localization_node,
            #gmapping_node,

            icp_odometry_node,
            points_xyzrgb_node,
            camera2base_node,
            #rs_camera_node,
            include_realsense2_camera,
            slam_toolbox_launch,
            slam_toolbox_node,
            nav2_launch,
            #nav2_node,

    ])