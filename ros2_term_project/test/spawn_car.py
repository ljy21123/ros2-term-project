#!/usr/bin/env python3

import os
import threading


def run_command():
    os.system("ros2 run gazebo_ros spawn_entity.py -file /home/ros2/Ros2Projects/oom_ws/src/ros2_term_project/models/prius_hybrid.sdf -entity PR001 -x 93 -y -11.7 -Y -1.57 -robot_namespace /PR001")
    os.system("ros2 run gazebo_ros spawn_entity.py -file /home/ros2/Ros2Projects/oom_ws/src/ros2_term_project/models/prius_hybrid.sdf -entity PR002 -x 93 -y -15.9 -Y -1.57 -robot_namespace /PR002")
    os.system("ros2 run ros2_term_project sub")


thread1 = threading.Thread(target=run_command)
thread1.start()
