# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    with_rviz2 = LaunchConfiguration('rviz2', default='true')
    with_nav2 = LaunchConfiguration('nav2', default='true')
    with_slam = LaunchConfiguration('slam', default='true')
    with_foxglove = LaunchConfiguration('foxglove', default='true')
    with_joystick = LaunchConfiguration('joystick', default='true')
    with_teleop = LaunchConfiguration('teleop', default='true')
    map_yaml_file = LaunchConfiguration('map', default='')

    launch_args = []
    launch_args.append(DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation (Gazebo) clock if true'))
    launch_args.append(DeclareLaunchArgument('rviz2', default_value='True', description='Launch RViz'))
    launch_args.append(DeclareLaunchArgument('nav2', default_value='True', description='Launch Navigation2'))
    launch_args.append(DeclareLaunchArgument('slam', default_value='True', description='Launch SLAM Toolbox if set to true, otherwise launch AMCL for localization'))
    launch_args.append(DeclareLaunchArgument('foxglove', default_value='True', description='Launch Foxglove Bridge'))
    launch_args.append(DeclareLaunchArgument('joystick', default_value='True', description='Launch Joystick'))
    launch_args.append(DeclareLaunchArgument('teleop', default_value='True', description='Launch Teleop'))
    launch_args.append(DeclareLaunchArgument('map', default_value='', description='Map file (.yaml) to load'))
    
    conn_type = os.getenv('CONN_TYPE', 'webrtc')
    use_livo_localization = os.getenv('LIVO_LOC', None)
    
    if use_livo_localization and conn_type != 'cyclonedds':
            raise RuntimeError("If 'LIVO_LOC' is set to false, you must use CycloneDDS as connection type.")
    
    def validate_params(context, *args, **kwargs):
        param1 = context.launch_configurations['slam']
        param2 = context.launch_configurations.get('map', None)
    
        if (param1 in ["False", "false"]) and not param2:
            raise RuntimeError("If 'slam' is set to false, you must specify a map file to load.")

        return []
    launch_args.append(OpaqueFunction(function=validate_params))
    

    robot_token = os.getenv('ROBOT_TOKEN', '') # how does this work for multiple robots?
    robot_ip = os.getenv('ROBOT_IP', '')
    robot_ip_lst = robot_ip.replace(" ", "").split(",")
    print("IP list:", robot_ip_lst)

    conn_mode = "single" if len(robot_ip_lst) == 1 else "multi"

    # these are debug only
    map_name = os.getenv('MAP_NAME', '3d_map')
    save_map = os.getenv('MAP_SAVE', 'true')


    if conn_mode == 'single':
        rviz_config = "single_robot_conf.rviz"
    else:
        rviz_config = "multi_robot_conf.rviz"

    if conn_type == 'cyclonedds':
        rviz_config = "cyclonedds_config.rviz"

    urdf_file_name = 'multi_go2.urdf'
    urdf = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        "urdf",
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_desc_modified_lst = []

    for i in range(len(robot_ip_lst)):
        robot_desc_modified_lst.append(robot_desc.format(robot_num=f"robot{i}"))

    urdf_launch_nodes = []

    joy_params = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config', 'joystick.yaml'
    )

    default_config_topics = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config', 'twist_mux.yaml')

    foxglove_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml',
    )

    slam_toolbox_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'mapper_params_online_async.yaml'
    )

    nav2_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'nav2_params.yaml' if not use_livo_localization else 'nav2_params_livo.yaml'
    )

    if conn_mode == 'single':

        urdf_file_name = 'go2_with_livox.urdf'
        urdf = os.path.join(
            get_package_share_directory('go2_robot_sdk'),
            "urdf",
            urdf_file_name)
        with open(urdf, 'r') as infp:
            robot_desc = infp.read()

        urdf_launch_nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                             'robot_description': robot_desc}],
                arguments=[urdf]
            ),
        )
        urdf_launch_nodes.append(
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan',
                remappings=[
                    ('cloud_in', 'point_cloud2'),
                    ('scan', 'scan'),
                ],
                parameters=[{
                    'target_frame': 'base_link',
                    'max_height': 0.5
                }],
                output='screen',
            ),
        )
        # urdf_launch_nodes.append(
        #     Node(
        #         package='pointcloud_to_laserscan',
        #         executable='pointcloud_to_laserscan_node',
        #         name='pointcloud_to_laserscan',
        #         remappings=[
        #             ('cloud_in', 'point_cloud2'),
        #             #  ('cloud_in', 'livox/lidar'),
        #             ('scan', 'scan'),
        #         ],
        #         parameters=[{
        #             'target_frame': 'base_link',
        #             'max_height': 0.1,
        #             'min_height': -0.2,
        #         }],
        #         output='screen',
        #     ),
        # ),
    
        # urdf_launch_nodes.append(
        #     Node(
        #         package='go2_robot_sdk',
        #         executable='sync_scan_time_node',
        #         name='sync_scan_time',
        #         parameters=[{
        #             'input_topic': '/scan',
        #             'output_topic': '/scan_synced',
        #         }],
        #         output='screen',
        #     ),
        # )

    else:

        for i in range(len(robot_ip_lst)):
            urdf_launch_nodes.append(
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    namespace=f"robot{i}",
                    parameters=[{'use_sim_time': use_sim_time,
                                 'robot_description': robot_desc_modified_lst[i]}],
                    arguments=[urdf]
                ),
            )
            urdf_launch_nodes.append(
                Node(
                    package='pointcloud_to_laserscan',
                    executable='pointcloud_to_laserscan_node',
                    name='pointcloud_to_laserscan',
                    remappings=[
                        ('cloud_in', f'robot{i}/point_cloud2'),
                        ('scan', f'robot{i}/scan'),
                    ],
                    parameters=[{
                        'target_frame': f'robot{i}/base_link',
                        'max_height': 0.1
                    }],
                    output='screen',
                ),
            )


    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        condition=IfCondition(with_rviz2),
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('go2_robot_sdk'), 'config', rviz_config), '--ros-args', '--log-level', 'warn'],
    )
    
    if use_livo_localization:
        # only launch map server. The localization part (map->odom transformation) is conducted by LIVO.
        map_server_node = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=rviz_node,
                on_start=[TimerAction(
                    period=5.0,
                    actions=[Node(
                        package='nav2_map_server',
                        executable='map_server',
                        name='map_server',
                        output='screen',
                        arguments=['--ros-args', '--log-level', 'INFO'],
                        parameters=[{
                            'yaml_filename': map_yaml_file,
                            'use_sim_time': use_sim_time,
                            'topic_name': 'occu_map',
                        }],
                    )]
                )]
            )
        )

        map_server_activation = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=rviz_node,
                on_start=[TimerAction(
                    period=6.0,
                    actions=[Node(
                        package='nav2_util',
                        executable='lifecycle_bringup',
                        name='map_server',
                        output='screen',
                        arguments=['map_server'],
                    )]
                )]
            )
        )
    

        localization_node = [map_server_node, map_server_activation]
    else:        
         # Map server should be launched after rviz process, otherwise rviz2 cannot receive the map topic as expected.
        # See https://get-help.theconstruct.ai/t/rviz-map-not-received-unit-2-ros2-navigation/30932/4 for detail.
        localization_node =  [RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=rviz_node,
                on_start=[TimerAction(
                    period=10.0,
                    actions=[IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')),
                        condition=IfCondition(PythonExpression(['not ', with_slam])),
                        launch_arguments={
                            'map': map_yaml_file,
                            'params_file': nav2_config,
                            'use_sim_time': use_sim_time,
                        }.items()
                    )]
                )]
            )
        )]
    
    return LaunchDescription([
        *launch_args,
        *urdf_launch_nodes,
        rviz_node,
        *localization_node,
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            parameters=[{'robot_ip': robot_ip, 'token': robot_token, "conn_type": conn_type, "use_livox_localization": use_livo_localization}],
        ),
        Node(
            package='go2_robot_sdk',
            executable='lidar_to_pointcloud',
            parameters=[{'robot_ip_lst': robot_ip_lst, 'map_name': map_name, 'map_save': save_map}],
        ),
        Node(
            package='joy',
            executable='joy_node',
            condition=IfCondition(with_joystick),
            parameters=[joy_params]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            condition=IfCondition(with_joystick),
            parameters=[default_config_topics],
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            condition=IfCondition(with_teleop),
            parameters=[
                {'use_sim_time': use_sim_time},
                default_config_topics
            ],
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(foxglove_launch),
            condition=IfCondition(with_foxglove),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'slam_toolbox'), 'launch', 'online_async_launch.py')
            ]),
            condition=IfCondition(with_slam),
            launch_arguments={
                'slam_params_file': slam_toolbox_config,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'nav2_bringup'), 'launch', 'navigation_launch.py')
            ]),
            condition=IfCondition(with_nav2),
            launch_arguments={
                'params_file': nav2_config,
                'use_sim_time': use_sim_time,
            }.items(),
            
        ),
    ])
