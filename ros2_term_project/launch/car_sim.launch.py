#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import json
from launch.substitutions import LaunchConfiguration


print(os.path.realpath(__file__))

ld = LaunchDescription()


def generate_launch_description():
    # print("차량을 선택해 주세요(PR001, PR002, PR003, PR004): ", end='')
    # while True:
    #     op = input()
    #     if op == 'PR001' or op == 'PR002' or op == 'PR003' or op == 'PR004':
    #         break
    #     else:
    #         print("다시 입력해주세요!")

    # configuration
    world = LaunchConfiguration('world')
    print('world =', world)
    world_file_name = 'car_track.world'
    world = os.path.join(get_package_share_directory('ros2_term_project'),
                         'worlds', world_file_name)
    print('world file name = %s' % world)
    # ld = LaunchDescription()
    declare_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    gazebo_run = ExecuteProcess(
        cmd=['gazebo', '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    ld.add_action(declare_argument)
    ld.add_action(gazebo_run)

    # 장애물 움직임 노드 실행
    cube_publisher_node = Node(
        namespace='/CUBE',
        package='ros2_term_project',
        executable='/home/ros2/Ros2Projects/oom_ws/src/ros2_term_project/ros2_term_project/cube_publisher.py',
        output='screen'
    )

    ld.add_action(cube_publisher_node)

    # gui_node = Node(
    #     namespace='/GUI',
    #     package='ros2_term_project',
    #     executable='/home/ros2/Ros2Projects/oom_ws/src/ros2_term_project/ros2_term_project/GUI.py',
    #     output='screen'
    # )

    # ld.add_action(gui_node)

    line_follower_node = Node(
        namespace='/PR001',
        package='ros2_term_project',
        executable='/home/ros2/Ros2Projects/oom_ws/src/ros2_term_project/ros2_term_project/line_follower.py',
        output='screen',
        arguments=['PR001']
    )

    ld.add_action(line_follower_node)

    line_follower_node2 = Node(
        namespace='/PR002',
        package='ros2_term_project',
        executable='/home/ros2/Ros2Projects/oom_ws/src/ros2_term_project/ros2_term_project/line_follower.py',
        output='screen',
        arguments=['PR002']
    )

    ld.add_action(line_follower_node2)

    # 자동차 소환
    spawn_car_node = Node(
        package='ros2_term_project',
        executable='/home/ros2/Ros2Projects/oom_ws/src/ros2_term_project/test/spawn_car.py',
        output='screen',
        # arguments=[op],
    )
    ld.add_action(spawn_car_node)

    return ld
