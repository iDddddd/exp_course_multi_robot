/*
 * Date: 2024-9-25
 * Description: 对群体机器人进行集体控制, 使其达到一致的角度,
 * 同时限制了机器人的角速度和线速度, 以及角度的变化阈值.
 * 程序在一个循环中不断检查角度是否已经收敛, 如果未收敛则调整机器人的角速度,
 * 直到达到一致. 最后, 程序停止所有机器人的运动并输出成功消息.
 */

#include <gazebo_swarm_robot_control.h>

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "swarm_robot_control_angle");

    // 机器人的id
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5};

    // 创建 SwarmRobot 对象
    SwarmRobot swarm_robot(swarm_robot_id);

    double conv_th = 0.05; // 角度跟踪的阈值(小于该值认为到达)(rad)

    /* Velocity scale and threshold */
    double MAX_W = 1;    // 最大角速度 (rad/s)
    double MIN_W = 0.05; // 最小角速度 (rad/s)
    // double MAX_V = 0.2;  // 最大线速度 (m/s)
    // double MIN_V = 0.01; // 最小线速度 (m/s)
    double k_w = 0.1; // 期望角速度 = k_w * del_theta, del_theta 为某机器人与其他机器人的角度差
    // double k_v = 0.1;

    // Laplace matrix
    Eigen::MatrixXd lap(swarm_robot.robot_num, swarm_robot.robot_num);
    lap << 4, -1, -1, -1, -1,
        -1, 4, -1, -1, -1,
        -1, -1, 4, -1, -1,
        -1, -1, -1, 4, -1,
        -1, -1, -1, -1, 4;

    /* 存储机器人当前位姿和与其他机器人位姿差的 Eigen 对象 */
    // Eigen::VectorXd cur_x(swarm_robot.robot_num);
    // Eigen::VectorXd cur_y(swarm_robot.robot_num);
    Eigen::VectorXd cur_theta(swarm_robot.robot_num);
    // Eigen::VectorXd del_x(swarm_robot.robot_num);
    // Eigen::VectorXd del_y(swarm_robot.robot_num);
    Eigen::VectorXd del_theta(swarm_robot.robot_num);

    // 机器人当前位姿
    std::vector<std::array<double, 3>> current_robot_pose(swarm_robot.robot_num);

    /* 运行直到各个机器人角度相同 */
    bool is_conv = false; // 是否到达
    while (!is_conv)
    {
        // 获取机器人当前位姿
        swarm_robot.getRobotPose(current_robot_pose);
        // 提取角度信息, 赋值给 cur_theta
        for (int i = 0; i < swarm_robot_id.size(); i++)
        {
            cur_theta(i) = current_robot_pose[i][2];
        }

        // 判断是否到达
        del_theta = -lap * cur_theta;
        is_conv = true;
        for (int i = 0; i < swarm_robot_id.size(); i++)
        {
            if (std::abs(del_theta(i)) > conv_th)
            {
                is_conv = false;
                break;
            }
        }

        if (is_conv)
        {
            break;
        }

        // 控制机器人运动
        for (int i = 0; i < swarm_robot.robot_num; i++)
        {
            double w = del_theta(i) * k_w;
            w = swarm_robot.checkVel(w, MAX_W, MIN_W);
            swarm_robot.moveRobot(i, 0.0, w);
        }

        // 等待一段时间
        ros::Duration(0.05).sleep();
    }

    // 停止所有机器人
    swarm_robot.stopRobot();

    ROS_INFO_STREAM("Succeed!");
    return 0;
}
