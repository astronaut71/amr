import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os



def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='amr_bot').find('amr_bot')
    #default_model_path = os.path.join(pkg_share, 'src/description/amr_bot_description.urdf')
    
    
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')],
    )

    camera2base_node = launch_ros.actions.Node(
        package = 'tf2_ros',
	    executable = 'static_transform_publisher',
	    arguments = ['-0.04','0', '0', '0', '0', '0', 'base_link', 'camera_link'],
    )


    return launch.LaunchDescription([
        #launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                          #  description='Flag to enable joint_state_publisher_gui'),
        #launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
           #                              description='Absolute path to robot urdf file'),
        #launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
         #                                   description='Absolute path to rviz config file'),
        #launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
         #                                   description='Flag to enable use_sim_time'),
        #launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        #joint_state_publisher_node,
        #robot_state_publisher_node,
        #spawn_entity,
        robot_localization_node,
        #camera2base_node,
        #rviz_node
    ])
