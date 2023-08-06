import launch
import launch_ros.actions

def generate_launch_description():
    namespace = launch.substitutions.LaunchConfiguration('namespace', default='rfans')
    output = launch.substitutions.LaunchConfiguration('output', default='screen')
    respawn = launch.substitutions.LaunchConfiguration('respawn', default='true')
    revise_angle_128 = launch.substitutions.LaunchConfiguration('revise_angle_128', default='0.027721,0.002361,-0.043092,-0.023711,0.003442,-0.046271,-0.018255,0.036872,-0.048702,-0.025516,0.002106,-0.040973,-0.019308,0.001663,-0.045102,-0.015489,0.001148,-0.047884,-0.018850,0.016742,-0.044528,-0.033280,0.001597,-0.039345,1,1.8,1,1.8,0.15,0.15,')
    revise_angle_32 = launch.substitutions.LaunchConfiguration('revise_angle_32', default='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,45,0,0,0,0.1,0,')
    read_fast = launch.substitutions.LaunchConfiguration('read_fast', default='false')
    read_once = launch.substitutions.LaunchConfiguration('read_once', default='false')
    repeat_delay = launch.substitutions.LaunchConfiguration('repeat_delay', default='0.0')
    frame_id = launch.substitutions.LaunchConfiguration('frame_id', default='surestar')

    rfans_driver_node = launch_ros.actions.Node(
        package='rfans_driver',
        executable='driver_node',
        name='rfans_driver',
        output=output,
        parameters=[
            {'advertise_name': 'surestar_packets'},
            {'control_name': 'surestar_control'},
            {'device_ip': '192.168.0.3'},
            {'device_port': 2014},
            {'rps': 10},
            {'pcap': ''},
            {'data_level': '3'},
            {'use_double_echo': 'false'},
            {'read_fast': read_fast},
            {'read_once': read_once},
            {'repeat_delay': repeat_delay},
        ]
    )

    calculation_node = launch_ros.actions.Node(
        package='rfans_driver',
        executable='calculation_node',
        name='calculation_node',
        parameters=[
            {'advertise_name': 'surestar_points'},
            {'subscribe_name': 'surestar_packets'},
            {'frame_id': frame_id},
            {'use_gps': 'false'},
            {'revise_angle_128': revise_angle_128},
            {'revise_angle_32': revise_angle_32},
        ]
    )

    cloud_process_node = launch_ros.actions.Node(
        package='rfans_driver',
        executable='cloud_process',
        name='cloud_process',
        remappings=[
            ('rfans_points', 'surestar_points'),
            ('/rfans_driver/rfans_points', 'rfans_driver/surestar_points'),
        ]
    )

    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', 'surestar_points'),
            ('scan', 'surestar_scan'),
        ]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('namespace', default_value=namespace),
        launch.actions.DeclareLaunchArgument('output', default_value=output),
        launch.actions.DeclareLaunchArgument('respawn', default_value=respawn),
        launch.actions.DeclareLaunchArgument('revise_angle_128', default_value=revise_angle_128),
        launch.actions.DeclareLaunchArgument('revise_angle_32', default_value=revise_angle_32),
        launch.actions.DeclareLaunchArgument('read_fast', default_value=read_fast),
        launch.actions.DeclareLaunchArgument('read_once', default_value=read_once),
        launch.actions.DeclareLaunchArgument('repeat_delay', default_value=repeat_delay),
        launch.actions.DeclareLaunchArgument('frame_id', default_value=frame_id),
        rfans_driver_node,
        calculation_node,
        cloud_process_node,
        pointcloud_to_laserscan_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
