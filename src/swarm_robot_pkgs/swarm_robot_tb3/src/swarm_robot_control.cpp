/*
 * Date: 2024-9-25
 * Description: the basic function
 */

#include <swarm_robot_control.h>

SwarmRobot::SwarmRobot(std::vector<int> swarm_robot_id_)
    : swarm_robot_id(swarm_robot_id_), speed(swarm_robot_id_.size())
{

    this->robot_num = swarm_robot_id.size();

    std::cout << "robot_num=" << robot_num << std::endl;
    /* 初始化控制命令发布者 */
    cmd_vel_pub.resize(this->robot_num);
    for (int i = 0; i < this->robot_num; i++)
    {
        // 话题名称类似于:/robot_1/cmd_vel
        std::string vel_topic = "/robot_" + std::to_string(swarm_robot_id_[i]) + "/cmd_vel";
        cmd_vel_pub[i] = nh_.advertise<geometry_msgs::Twist>(vel_topic, 10);
    }
}

SwarmRobot::~SwarmRobot()
{
}

bool SwarmRobot::getRobotPose(int index, std::array<double, 3> &pose_cur)
{
    tf::StampedTransform transform;
    std::string robot_frame = "robot_" + std::to_string(this->swarm_robot_id[index]);
    std::string base_marker = "base_marker";

    // 获取机器人的位姿
    try
    {
        this->tf_listener.waitForTransform(base_marker, robot_frame, ros::Time(0), ros::Duration(0.5));
        this->tf_listener.lookupTransform(base_marker, robot_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        // ros::Duration(1.0).sleep();
        return false;
    }

    tf::Quaternion q = transform.getRotation();
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // pose_cur 仅包含 x, y, yaw
    pose_cur[0] = transform.getOrigin().x();
    pose_cur[1] = transform.getOrigin().y();
    pose_cur[2] = yaw;

    // ROS_INFO_STREAM("Get pose of robot_" << swarm_robot_id[index] << " is: x=" << pose_cur[0] << " y=" << pose_cur[1] << " theta=" << pose_cur[2]);
    return true;
}

bool SwarmRobot::getRobotPose(std::vector<std::array<double, 3>> &current_robot_pose)
{
    current_robot_pose.resize(this->robot_num);
    std::vector<bool> flag_pose(this->robot_num, false);

    // 获取机器人的位姿, 直到所有机器人的位姿都获取到
    bool flag = false;
    while (!flag)
    {
        flag = true;
        for (int i = 0; i < this->robot_num; i++)
        {
            flag = flag && flag_pose[i];
        }
        for (int i = 0; i < this->robot_num; i++)
        {
            std::array<double, 3> pose_robot;
            if (getRobotPose(i, pose_robot))
            {
                current_robot_pose[i] = pose_robot;
                flag_pose[i] = true;
            }
        }
    }
    // ROS_INFO_STREAM("Succeed getting pose!");
    return true;
}

void SwarmRobot::getRobotSpeed(int index, std::array<double, 2> &speed)
{
    speed[0] = this->speed[index][0];
    speed[1] = this->speed[index][1];
}

void SwarmRobot::getRobotSpeed(std::vector<std::array<double, 2>> &swarm_speed)
{
    swarm_speed.resize(this->robot_num);
    for (int i = 0; i < this->robot_num; i++)
    {
        getRobotSpeed(i, swarm_speed[i]);
    }
}

bool SwarmRobot::moveRobot(int index, double v, double w)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = v;
    vel_msg.angular.z = w;
    this->speed[index][0] = v;
    this->speed[index][1] = w;
    cmd_vel_pub[index].publish(vel_msg);
    // ROS_INFO_STREAM("Move robot_" << swarm_robot_id[index] << " with v=" << v << " w=" << w);
    return true;
}

bool SwarmRobot::moveRobot(std::vector<std::array<double, 2>> &speed)
{
    if (this->robot_num != speed.size())
    {
        ROS_ERROR_STREAM("The robot number does not equal the speed number!");
        return false;
    }

    for (int i = 0; i < this->robot_num; i++)
    {
        if (!this->moveRobot(i, speed[i][0], speed[i][1]))
        {
            return false;
        }
    }
    return true;
}

bool SwarmRobot::stopRobot(int index)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    this->speed[index][0] = 0.0;
    this->speed[index][1] = 0.0;
    cmd_vel_pub[index].publish(vel_msg);
    ROS_INFO_STREAM("Stop robot_" << swarm_robot_id[index]);
    return true;
}

bool SwarmRobot::stopRobot()
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    for (int i = 0; i < this->robot_num; i++)
    {
        cmd_vel_pub[i].publish(vel_msg);
    }
    ROS_INFO_STREAM("Stop all robots.");
    return true;
}

double SwarmRobot::checkVel(double v, double max_v, double min_v)
{
    if (max_v <= 0 || min_v <= 0)
    {
        std::cout << "Error input of checkVel()" << std::endl;
        return v;
    }

    if (v > 0)
    {
        v = std::max(v, min_v);
        v = std::min(v, max_v);
    }
    else
    {
        v = std::min(v, -min_v);
        v = std::max(v, -max_v);
    }
    return v;
}

void SwarmRobot::U2VW(int index, double ux, double uy, double &v, double &w)
{
    const double pi = 3.141592653589793;
    v = 0;
    w = 0;

    // 获取当前机器人的姿态
    std::array<double, 3> pose_cur;
    getRobotPose(index, pose_cur);
    double theta_robot = pose_cur[2];

    /* 速度 */
    // 速度大小
    double v0 = std::sqrt(ux * ux + uy * uy);
    // 速度方向
    double theta_v = std::atan2(uy, ux);

    // 限定 angle 大小为 [-pi, pi]
    double angle = theta_v - theta_robot;
    while (angle > pi || angle < -pi)
    {
        if (angle > pi)
        {
            angle = angle - 2 * pi;
        }
        else
        {
            angle = angle + 2 * pi;
        }
    }

    double W = 1; // 角速度最大值参考参数
    double V = 1; // 速度方向
    // 判断速度方向(取消注释允许机器人反向运动)
    // if (angle > pi / 2)
    // {
    //     angle = angle - pi;
    //     // 速度反向
    //     V = -1;
    // }
    // else if (angle < -pi / 2)
    // {
    //     angle = pi - angle;
    //     // 速度反向
    //     V = -1;
    // }
    // else
    // {
    //     // 速度正向
    //     V = 1;
    // }

    // 计算速度
    w = W * (angle / std::abs(angle)) * (std::exp(std::abs(angle)) - 1);
    v = V * v0 * std::exp(-std::abs(angle));
    if (v > 0 && v > 0.5)
    {
        v = 0.5;
    }
    else if (v < 0 && v < -0.5)
    {
        v = -0.5;
    }
    else if (v == 0)
    {
        w = 0;
    }

    // std::cout << "//angle = " << angle << endl;
    // std::cout << "//theta_v = " << theta_v << endl;
    // std::cout << "//theta_robot = " << theta_robot << endl;
    // std::cout << "//v = " << v << endl;
    // std::cout << "//w = " << w << endl;
}

void SwarmRobot::moveRobotbyU(int index, double ux_0, double uy_0)
{
    double v = 0;
    double w = 0;
    // ux_0 *= 0.5;
    // uy_0 *= 0.5;
    U2VW(index, ux_0, uy_0, v, w);
    v = this->checkVel(v, MAX_V, MIN_V);
    w = this->checkVel(w, MAX_W, MIN_W);
    moveRobot(index, v, w);
}

void SwarmRobot::moveRobotsbyU(Eigen::VectorXd del_x, Eigen::VectorXd del_y)
{
    // for (int i = 0; i < this->robot_num; i++) {
    //     moveRobotbyU(i, del_x(i), del_y(i));
    // }
    // double v, w;

    // Eigen::VectorXd v_theta(this->robot_num);
    // Eigen::VectorXd del_theta(this->robot_num);
    // Eigen::VectorXd cur_theta(this->robot_num);
    // std::vector<std::array<double, 3>> current_robot_pose(this->robot_num);
    // double pi = 3.14159;
    // this->getRobotPose(current_robot_pose);
    for (int i = 0; i < this->robot_num; i++)
    {
        this->moveRobotbyU(i, del_x(i), del_y(i));
    }
    // for(int i = 0; i < this->robot_num; i++) {
    //     cur_theta(i) = current_robot_pose[i][2]; // 提取角度信息
    // }

    // for (int i = 0; i < this->robot_num; i++) {
    //         v_theta(i) = std::atan2(del_y(i) , del_x(i));
    //         del_theta(i) = -(cur_theta(i) - v_theta(i));
    //         while (del_theta(i) < -pi or del_theta(i) > pi) {
    //             if (del_theta(i) < -pi) del_theta(i) += 2 * pi;
    //             if (del_theta(i) > pi) del_theta(i) -= 2 * pi;
    //         }
    //     }
    //     /* Swarm robot move */
    // for(int i = 0; i < this->robot_num; i++) {
    //     if (std::fabs(del_theta(i)) > 0.1) {
    //         w = del_theta(i) / std::fabs(del_theta(i)) * MAX_W;
    //         v = 0;
    //     }
    //     else {
    //         w = del_theta(i) / std::fabs(del_theta(i)) * MIN_W;
    //         v = std::sqrt(std::pow(del_x(i),2) + std::pow(del_y(i),2));
    //         v = this->checkVel(v, MAX_V, MIN_V);
    //     }
    //     this->moveRobot(i, v, w);
    // }
    // ros::Duration(0.05).sleep();
}
