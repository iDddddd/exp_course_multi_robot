#! /usr/bin/env python3
# encoding: utf-8
import numpy as np
import rospy
from gazebo_swarm_robot_control import SwarmRobot


def main():
    # 初始化节点
    rospy.init_node("swarm_robot_control_angle")
    # 机器人的id
    index = [1, 2, 3, 4, 5]
    # 建立对象
    swarm_robot = SwarmRobot(index)

    conv_ang = .05  # convergence angle
    conv_pos = .01  # convergence position
    MAX_W = 1  # 最大角速度 (rad/s)
    MIN_W = 0.05  # 最小角速度 (rad/s)
    MAX_V = 0.2  # 最大线速度 (m/s)
    MIN_V = 0.01  # 最小线速度 (m/s)
    k_w = 0.1  # w = k_w * ang_diff
    k_v = 0.1

    # Laplace matrix
    lap = np.array(
        [
            [4, -1, -1, -1, -1],
            [-1, 4, -1, -1, -1],
            [-1, -1, 4, -1, -1],
            [-1, -1, -1, 4, -1],
            [-1, -1, -1, -1, 4],
        ]
    )

    # Save current pos/ang, target and difference 
    cur_x = np.array([0.0] * swarm_robot.robot_num)
    cur_y = np.array([0.0] * swarm_robot.robot_num)
    cur_theta = np.array([0.0] * swarm_robot.robot_num)

    tar_x = np.array([0.0] * swarm_robot.robot_num)
    tar_y = np.array([0.0] * swarm_robot.robot_num)
    tar_theta = np.array([0.0] * swarm_robot.robot_num)

    ang_diff = np.array([1.0] * swarm_robot.robot_num)
    pos_diff = np.array([1.0] * swarm_robot.robot_num)

    # Get current pos/ang off all
    current_robot_pose = swarm_robot.get_robot_poses()
    print(current_robot_pose)

    current_robot_pose = swarm_robot.get_robot_poses()
    print(current_robot_pose) 

    # Calculate center point
    avg_x = 0.0
    avg_y = 0.0
    for i in range(swarm_robot.robot_num):
        cur_x[i] = current_robot_pose[i][0]
        cur_y[i] = current_robot_pose[i][1]
        cur_theta[i] = current_robot_pose[i][2]
        avg_x = avg_x + cur_x[i]
        avg_y = avg_y + cur_y[i]
        print('cur', i, cur_x[i], cur_y[i], cur_theta[i])

    avg_x = avg_x / swarm_robot.robot_num
    avg_y = avg_y / swarm_robot.robot_num
    print('avg', avg_x, avg_y)

    xshift = .5
    yshift = .5

    # Set target position based on current x pos
    for i in range(swarm_robot.robot_num):
        index = np.argsort(cur_x)[i]
        tar_x[index] = avg_x + (i - 2) * xshift
        tar_y[index] = avg_y + yshift
        
    # Calculate the direction to target
    for i in range(swarm_robot.robot_num):
        tar_theta[i] = np.arctan2(tar_y[i] - cur_y[i], tar_x[i] - cur_x[i])
        print('tar', i, tar_x[i], tar_y[i], tar_theta[i])

    ang_conv = False
    pos_conv = False

    # First turn to the direction to target
    while not ang_conv:
        for i in range(swarm_robot.robot_num):
            current_robot_pose = swarm_robot.get_robot_poses()
            cur_theta[i] = current_robot_pose[i][2]
            cur_x[i] = current_robot_pose[i][0]
            cur_y[i] = current_robot_pose[i][1]
            tar_theta[i] = np.arctan2(tar_y[i] - cur_y[i], tar_x[i] - cur_x[i])

            ang_diff[i] = tar_theta[i] - cur_theta[i]
            
            print(i, 'ang_diff', ang_diff[i])
                    
            ang_conv = np.all(np.abs(ang_diff) <= conv_ang)
    

            if ang_conv:
                break 
            
            
            if abs(ang_diff[i]) > conv_ang:
                w = k_w * ang_diff[i]
                w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                swarm_robot.move_robot(i, 0, w)
                print(i, 'w', w)
 
        rospy.sleep(0.05)

    # Then reach for target pos w/ ang fixing
    while not pos_conv:  
        for i in range(swarm_robot.robot_num):
            current_robot_pose = swarm_robot.get_robot_poses()
            cur_theta[i] = current_robot_pose[i][2]
            cur_x[i] = current_robot_pose[i][0]
            cur_y[i] = current_robot_pose[i][1]
            tar_theta[i] = np.arctan2(tar_y[i] - cur_y[i], tar_x[i] - cur_x[i])

            ang_diff[i] = tar_theta[i] - cur_theta[i]
            pos_diff[i] = np.sqrt(
                (tar_y[i] - cur_y[i]) ** 2 + (tar_x[i] - cur_x[i]) ** 2
            )

            print(i, 'pos_diff', pos_diff[i])

            pos_conv = np.all(pos_diff <= conv_pos)
            print('pos_conv', pos_conv)

            if pos_conv:
                break

            if pos_diff[i] > conv_pos:
                v = k_v * pos_diff[i]
                v = swarm_robot.check_vel(v, MAX_V, 0)
                w = k_w * ang_diff[i]
                w = swarm_robot.check_vel(w, MAX_W, 0)
                swarm_robot.move_robot(i, v, w)  
                print(i, 'v', v, 'w', w) 

        rospy.sleep(0.05)

    same_ang = 0.0
    ang_same = False
    while not ang_same:
        for i in range(swarm_robot.robot_num):
            current_robot_pose = swarm_robot.get_robot_poses()
            cur_theta[i] = current_robot_pose[i][2]
            
            ang_diff[i] = same_ang - cur_theta[i]
            
            print(i, 'ang_diff', ang_diff[i])  
            
            ang_same = np.all(np.abs(ang_diff) <= conv_ang)
            print('ang_same', ang_same)

            if ang_same:
                break
            
            if abs(ang_diff[i]) > conv_ang:
                w = k_w * ang_diff[i]
                w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                swarm_robot.move_robot(i, 0, w)
                print(i, 'w', w)
 
        rospy.sleep(0.05)        

    # 停止所有机器人
    swarm_robot.stop_robots()

    rospy.loginfo("Succeed!")


if __name__ == "__main__":
    main()
