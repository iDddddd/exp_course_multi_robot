#! /usr/bin/env python3
# encoding: utf-8
import rospy
import tf
from geometry_msgs.msg import Twist
import numpy as np


class SwarmRobot:

    MAX_W = 1  # 最大角速度(rad/s)
    MIN_W = 0.05  # 最小角速度(rad/s)
    MAX_V = 0.2  # 最大线性速度(m/s)
    MIN_V = 0.01  # 最小线性速度(m/s)

    def __init__(self, swarm_robot_id: list):
        """
        多移动机器人基本控制类

        Args:
            swarm_robot_id: 各个机器人的ID
        """

        self.swarm_robot_id = swarm_robot_id  # 各个机器人的ID
        self.robot_num = len(swarm_robot_id)  # 机器人总数

        # 机器人控制命令发布者
        self.cmd_vel_pub = [
            rospy.Publisher(
                "/robot_{}/cmd_vel".format(swarm_robot_id[i]), Twist, queue_size=10
            )
            for i in range(self.robot_num)
        ]
        # ROS tf listener
        self.tf_listener = tf.TransformListener()

        # 机器人移动速度
        self.speed = [[0.0, 0.0] for _ in range(self.robot_num)]

    def get_robot_pose(self, index: int) -> tuple:
        """
        获取单个机器人的位姿

        Args:
            index: 机器人索引

        Returns:
            - 是否u获取成功
            - 机器人的位姿, 包含 x, y, yaw
        """

        pose_cur = [0.0, 0.0, 0.0]
        robot_frame = "robot_{}/base_footprint".format(self.swarm_robot_id[index])
        base_marker = "base_marker"

        # 获取机器人的位姿
        try:
            self.tf_listener.waitForTransform(
                base_marker, robot_frame, rospy.Time(0), rospy.Duration(0.5)
            )
            (trans, rot) = self.tf_listener.lookupTransform(
                base_marker, robot_frame, rospy.Time(0)
            )
        except Exception as ex:
            rospy.logerr_once(str(ex))
            return False, None

        _, _, yaw = tf.transformations.euler_from_quaternion(rot)
        pose_cur[0] = trans[0]
        pose_cur[1] = trans[1]
        pose_cur[2] = yaw
        # rospy.loginfo(
        #     f"Get pose of robot_{self.swarm_robot_id[index]}: x={pose_cur[0]} y={pose_cur[1]} theta={pose_cur[2]}"
        # )
        return True, pose_cur

    def get_robot_poses(self) -> list:
        """
        获取所有机器人的位姿

        Returns:
            所有机器人的位姿
        """
        # 获取机器人的位姿, 直到所有机器人的位姿都获取到
        current_robot_pose = []
        self.flag_pose = [False for _ in range(self.robot_num)]
        while True:
            flag = all(self.flag_pose)
            if flag:
                break

            for i in range(self.robot_num):
                success, pose_robot = self.get_robot_pose(i)
                if success:
                    current_robot_pose.append(pose_robot)
                    self.flag_pose[i] = True

        # rospy.loginfo("Succeed getting pose!")
        return current_robot_pose

    def get_robot_speed(self, index: int) -> list:
        """
        获取单个机器人的速度

        Args:
            index: 机器人索引

        Returns:
            机器人的速度, 包含线速度和角速度
        """
        return self.speed[index]

    def get_robot_speeds(self) -> list:
        """
        获取所有机器人的速度

        Returns:
            所有机器人的速度
        """
        return self.speed

    def move_robot(self, index, v, w) -> bool:
        """
        移动第 `index` 个机器人

        Args:
            index: 机器人的序号
            v: 机器人的线速度
            w: 机器人的角速度
        """
        vel_msg = Twist()
        vel_msg.linear.x = v
        vel_msg.angular.z = w
        self.cmd_vel_pub[index].publish(vel_msg)
        # rospy.loginfo(f"Move robot_{self.swarm_robot_id[index]} with v={v} w={w}")
        return True

    def move_robots(self, speed: list) -> bool:
        """
        移动所有机器人

        Args:
            speed: 机器人的速度, 包含线速度和角速度

        Returns:
            是否移动成功
        """
        if len(speed) != self.robot_num:
            rospy.logerr("The robot number does not equal the speed number!")
            return False

        for i in range(self.robot_num):
            if not self.move_robot(i, speed[i][0], speed[i][1]):
                return False
        return True

    def stop_robot(self, index):
        """
        停止单个机器人

        Args:
            index: 机器人的序号
        """
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_vel_pub[index].publish(vel_msg)
        # rospy.loginfo(f"Stop robot_{self.swarm_robot_id[index]}")
        return True

    def stop_robots(self):
        """
        停止所有机器人
        """
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        for i in range(self.robot_num):
            self.cmd_vel_pub[i].publish(vel_msg)
        rospy.loginfo("Stop all robots.")
        return True

    @staticmethod
    def check_vel(v, max_v, min_v):
        """
        将机器人的速度限制在最大值和最小值之间
        """
        if max_v <= 0 or min_v <= 0:
            rospy.loginfo("Error input of check_vel()")
            return v

        if v > 0:
            v = max(min(v, max_v), min_v)
        else:
            v = min(max(v, -min_v), -max_v)
        return v

    def u2vw(self, index, ux, uy) -> list:
        """
        将机器人的期望速度转换为线速度和角速度

        Args:
            index: 机器人的序号
            ux: 机器人的期望速度x
            uy: 机器人的期望速度y

        Returns:
            机器人的线速度和角速度 [v, w]
        """

        # 获取当前机器人的姿态
        success, pose_cur = self.get_robot_pose(index)
        if not success:
            return [0, 0]
        theta_robot = pose_cur[2]

        # 速度大小
        v0 = np.sqrt(ux * ux + uy * uy)
        # 速度方向
        theta_v = np.arctan2(uy, ux)

        # 限定 angle 大小为 [-pi, pi]
        angle = theta_v - theta_robot
        while angle > np.pi or angle < -np.pi:
            if angle > np.pi:
                angle = angle - 2 * np.pi
            else:
                angle = angle + 2 * np.pi

        W = 1  # 角速度最大值参考参数
        V = 1  # 速度方向

        # 判断速度方向(取消注释允许机器人反向运动)
        # if angle > np.pi / 2:
        #     angle = angle - np.pi
        #     # 速度反向
        #     V = -1
        # elif angle < -np.pi / 2:
        #     angle = np.pi - angle
        #     # 速度反向
        #     V = -1
        # else:
        #     # 速度正向
        #     V = 1

        # 计算速度
        w = W * (angle / np.abs(angle)) * (np.exp(np.abs(angle)) - 1)
        v = V * v0 * np.exp(-np.abs(angle))
        if v > 0 and v > 0.5:
            v = 0.5
        elif v < 0 and v < -0.5:
            v = -0.5
        elif v == 0:
            w = 0

        return [v, w]

    def move_robot_by_u(self, index, ux_0, uy_0):
        """
        移动第 `index` 个机器人

        Args:
            index: 机器人的序号
            ux_0: 机器人的期望速度x
            uy_0: 机器人的期望速度y
        """

        v, w = self.u2vw(index, ux_0, uy_0)
        v = self.check_vel(v, self.MAX_V, self.MIN_V)
        w = self.check_vel(w, self.MAX_W, self.MIN_W)
        self.move_robot(index, v, w)

    def move_robots_by_u(self, ux, uy):
        """
        移动所有机器人

        Args:
            ux: 机器人的期望速度x
            uy: 机器人的期望速度y
        """
        for i in range(self.robot_num):
            self.move_robot_by_u(i, ux[i], uy[i])
