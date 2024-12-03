#! /usr/bin/env python3
# encoding: utf-8
"""
Date: 2024-10-20
Description: Control the swarm robot to achieve consistent speed.
"""

import rospy
import numpy as np
from gazebo_swarm_robot_control import SwarmRobot


def main():
    # 初始化节点
    rospy.init_node("swarm_robot_control_formation")
    # 机器人的id
    index = [1, 2, 3, 4, 5]
    # 建立对象
    swarm_robot = SwarmRobot(index)

    # 速度跟踪的阈值(v-sqrt(ux^2+uy^2)小于该值认为到达)(m/s)
    conv_v = 0.01
    # 角度跟踪的阈值(机器人姿态 w 小于该值认为到达)(rad)
    conv_w = 0.01

    # 目标速度
    target_speed = [0.1, 0.1]
    del_x = [target_speed[0]] * swarm_robot.robot_num
    del_y = [target_speed[1]] * swarm_robot.robot_num

    # 机器人当前位姿
    current_robot_pose = [[0.0, 0.0, 0.0] for _ in range(swarm_robot.robot_num)]
    # 机器人当前速度
    current_robot_speed = [[0.0, 0.0] for _ in range(swarm_robot.robot_num)]

    # 运行直到各个机器人速度达到目标速度
    is_conv = False  # 是否到达
    while not is_conv:
        # 获取机器人当前位姿
        current_robot_pose = swarm_robot.get_robot_poses()
        # 获取机器人当前速度
        current_robot_speed = swarm_robot.get_robot_speeds()

        # 判断是否到达
        is_conv = True
        for i in range(swarm_robot.robot_num):
            ux = target_speed[0] - current_robot_speed[i][0]
            uy = target_speed[1] - current_robot_speed[i][1]
            v = np.sqrt(ux * ux + uy * uy)
            w = np.abs(np.arctan2(uy, ux) - current_robot_pose[i][2])
            if v > conv_v or w > conv_w:
                is_conv = False
                break

        # 移动机器人
        swarm_robot.move_robots_by_u(del_x, del_y)

        # 等待一段时间
        rospy.sleep(0.05)

    rospy.loginfo("Succeed!")


if __name__ == "__main__":
    main()
