/*
 * Date: 2024-10-20
 * Description: 对群体机器人进行集体控制, 使其达到一致的速度.
 */

#include <swarm_robot_control.h>

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "swarm_robot_control_u");

    // 机器人的id
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5};

    // 创建 SwarmRobot 对象
    SwarmRobot swarm_robot(swarm_robot_id);

    double conv_v = 0.01; // 速度跟踪的阈值(v-sqrt(ux^2+uy^2)小于该值认为到达)(m/s)
    double conv_w = 0.01; // 角度跟踪的阈值(机器人姿态 w 小于该值认为到达)(rad)

    // 目标速度
    std::array<double, 2> target_speed{0.1, 0.1};
    Eigen::VectorXd del_x(swarm_robot.robot_num);
    Eigen::VectorXd del_y(swarm_robot.robot_num);
    for (int i = 0; i < swarm_robot.robot_num; i++)
    {
        del_x(i) = target_speed[0];
        del_y(i) = target_speed[1];
    }

    // 机器人当前位姿
    std::vector<std::array<double, 3>> current_robot_pose(swarm_robot.robot_num);
    // 机器人当前速度
    std::vector<std::array<double, 2>> current_robot_speed(swarm_robot.robot_num);

    /* 运行直到各个机器人速度达到目标速度 */
    bool is_conv = false; // 是否到达
    while (!is_conv)
    {
        // 获取机器人当前位姿
        swarm_robot.getRobotPose(current_robot_pose);
        // 获取机器人当前速度
        swarm_robot.getRobotSpeed(current_robot_speed);

        // 判断是否到达
        is_conv = true;
        for (int i = 0; i < swarm_robot.robot_num; i++)
        {
            double ux = target_speed[0] - current_robot_speed[i][0];
            double uy = target_speed[1] - current_robot_speed[i][1];
            double v = std::sqrt(ux * ux + uy * uy);
            double w = std::abs(std::atan2(uy, ux) - current_robot_pose[i][2]);
            if (v > conv_v || w > conv_w)
            {
                is_conv = false;
                break;
            }
        }

        // 移动机器人
        swarm_robot.moveRobotsbyU(del_x, del_y);

        // 等待一段时间
        ros::Duration(0.05).sleep();
    }

    ROS_INFO_STREAM("Succeed!");
    return 0;
}
