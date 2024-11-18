/*
 * Date: 2021-11-29
 * Description: the basic function
 */

#include <swarm_robot_control.h>

SwarmRobot::SwarmRobot(std::vector<int> swarm_robot_id_) : swarm_robot_id(swarm_robot_id_)
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
    // 坐标系名称与仿真中不同
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

    ROS_INFO_STREAM("Get pose of robot_" << swarm_robot_id[index] << " is: x=" << pose_cur[0] << " y=" << pose_cur[1] << " theta=" << pose_cur[2]);
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
    // 判断速度方向
    if (angle > pi / 2)
    {
        angle = angle - pi;
        // 速度反向
        V = -1;
    }
    else if (angle < -pi / 2)
    {
        angle = pi - angle;
        // 速度反向
        V = -1;
    }
    else
    {
        // 速度正向
        V = 1;
    }

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

void SwarmRobot::Formation(Eigen::VectorXd needed_x, Eigen::VectorXd needed_y, Eigen::MatrixXd lap, double conv_x, double conv_y)
{
    bool is_conv = false;
    std::cout << "this->robot_num" << this->robot_num << endl;
    std::vector<std::array<double, 3>> current_robot_pose(this->robot_num);
    Eigen::VectorXd del_x(this->robot_num);
    Eigen::VectorXd del_y(this->robot_num);
    // Eigen::VectorXd del_theta(this->robot_num);
    // Eigen::VectorXd v_theta(this->robot_num);
    Eigen::VectorXd cur_x(this->robot_num);
    Eigen::VectorXd cur_y(this->robot_num);
    Eigen::VectorXd cur_theta(this->robot_num);
    double k_w = 0.1; // 角速度的缩放比例
    double k_v = 0.1; // 线性速度的缩放比例
    double w, v;
    double pi = 3.1415926;

    while (!is_conv)
    { // 当未达到收敛条件时执行以下代码

        this->getRobotPose(current_robot_pose);

        for (int i = 0; i < this->robot_num; i++)
        {
            cur_x(i) = current_robot_pose[i][0];     // 提取位置信息
            cur_y(i) = current_robot_pose[i][1];     // 提取位置信息
            cur_theta(i) = current_robot_pose[i][2]; // 提取角度信息
        }

        /* 判断是否达到收敛条件 */
        del_x = -lap * (cur_x + needed_x); // 计算需要的x的变化
        del_y = -lap * (cur_y + needed_y); // 计算需要的y的变化
        is_conv = true;                    // 假设已经达到收敛条件
        for (int i = 0; i < this->robot_num; i++)
        {
            // cout << del_x(i) << " " << del_y(i) << endl;
            if ((std::abs(del_x(i)) > conv_x) or (std::abs(del_y(i)) > conv_y))
            {
                is_conv = false; // 如果任何一个x坐标的变化大于阈值,则认为未收敛
            }
        }

        this->moveRobotsbyU(del_x, del_y);

        ros::Duration(0.05).sleep();
    }
    cout << "Suceessfully form formation" << endl;
    this->stopRobot();
    ros::Duration(2).sleep();
}

void SwarmRobot::MoveFormation(Eigen::MatrixXd Gd0, Eigen::MatrixXd lap, double v_x, double v_y)
{
    // 在形成编队之后,移动编队
    std::cout << "this->robot_num" << this->robot_num << endl;
    std::vector<std::vector<double>> current_robot_pose(this->robot_num);
    Eigen::VectorXd del_x(this->robot_num);
    Eigen::VectorXd del_y(this->robot_num);
    Eigen::VectorXd del_theta(this->robot_num);
    Eigen::VectorXd v_theta(this->robot_num);
    Eigen::VectorXd cur_x(this->robot_num);
    Eigen::VectorXd cur_y(this->robot_num);
    Eigen::VectorXd cur_theta(this->robot_num);

    Eigen::MatrixXd Gx(swarm_robot_id.size(), swarm_robot_id.size());
    Eigen::MatrixXd Gy(swarm_robot_id.size(), swarm_robot_id.size());
    Eigen::MatrixXd Gd(swarm_robot_id.size(), swarm_robot_id.size());
    /*initialize the g*/
    Eigen::MatrixXd gx(this->robot_num, this->robot_num);
    Eigen::MatrixXd gy(this->robot_num, this->robot_num);

    double k_w = 0.1; // 角速度的缩放比例
    double k_v = 0.1; // 线性速度的缩放比例
    double pi = 3.1415926;

    /*initialize the robot speed*/
    double ux[this->robot_num] = {0};
    double uy[this->robot_num] = {0};

    // /*initialize the G0*/
    // Eigen::MatrixXd Gd0(this->robot_num, this->robot_num); // 创建Gy0矩阵对象
    Eigen::MatrixXd gd(this->robot_num, this->robot_num); // 创建Gy0矩阵对象
    // /*初始化速度*/
    // for (int i = 0; i < this->robot_num; i++)
    // {
    //     std::cout << "ux" << i << "=" << ux[i] << endl;
    //     std::cout << "uy" << i << "=" << uy[i] << endl;
    //     ux[i] = 0;
    //     uy[i] = 0;
    // }

    /*获得当前距离矩阵*/
    this->GetGxGyGd(Gx, Gy, Gd);

    /*计算获得保持队形需要速度*/
    this->GetVelocity(Gd0, Gd, ux, uy);

    /*叠加直线运动速度*/
    // 给每个机器人添加一个相同的速度
    for (int i = 0; i < this->robot_num; i++)
    {
        ux[i] += v_x;
        uy[i] += v_y;
    }
    // ux[0] += v_x;
    // uy[0] += v_y;
    // ux[1] += v_x;
    // uy[1] += v_y;

    for (int i = 0; i < this->robot_num; i++)
    {
        this->moveRobotbyU(i, ux[i], uy[i]);
    }
    ros::Duration(0.05).sleep();
}

void SwarmRobot::ChangeFormationDirection(double target_direction)
{
    // 用pid控制方法,调整每个机器人到制定角度target_direction
    // 用一个while循环,每次循环都检查是否达到目标角度
    // 如果没有达到目标角度,则继续调整
    // 如果达到目标角度,则停止调整
    bool is_conv = false;
    std::vector<std::array<double, 3>> current_robot_pose(this->robot_num);
    getRobotPose(current_robot_pose);
    Eigen::VectorXd del_theta(this->robot_num);
    Eigen::VectorXd cur_theta(this->robot_num);
    double MAX_W = 1;    // 最大角速度(弧度/秒)
    double MIN_W = 0.05; // 最小角速度(弧度/秒)
    for (int i = 0; i < this->robot_num; i++)
    {
        cur_theta(i) = current_robot_pose[i][2]; // 提取角度信息
    }
    // PID控制器
    double Kp = 1;
    double Ki = 0.01;
    double Kd = 0.01;
    double error = 0;
    double error_last = 0;
    double error_sum = 0;
    double w = 0;
    double v = 0;
    double pi = 3.1415926;

    while (!is_conv)
    { // 当未达到收敛条件时执行以下代码
        /* 判断是否达到收敛条件 */
        for (int i = 0; i < this->robot_num; i++)
        {
            del_theta(i) = -(cur_theta(i) - target_direction);
            while (del_theta(i) < -pi or del_theta(i) > pi)
            {
                if (del_theta(i) < -pi)
                    del_theta(i) += 2 * pi;
                if (del_theta(i) > pi)
                    del_theta(i) -= 2 * pi;
            }
            error = del_theta(i);
            w = Kp * error;
            this->moveRobot(i, v, w);
        }
        ros::Duration(0.05).sleep();
        // error = del_theta.sum();
        // error_sum += error;
        // w = Kp * error + Ki * error_sum + Kd * (error - error_last);
        // error_last = error;
        getRobotPose(current_robot_pose);
        for (int i = 0; i < this->robot_num; i++)
        {
            cur_theta(i) = current_robot_pose[i][2]; // 提取角度信息
        }
        for (int i = 0; i < this->robot_num; i++)
        {
            if (std::abs(del_theta(i)) > 0.2)
            {
                is_conv = false;
                break;
            }
            else
            {
                is_conv = true;
            }
        }
    }
    this->stopRobot();
    ros::Duration(2).sleep();
}

void SwarmRobot::ComeDot(int index, double x0, double y0, double &ux, double &uy)
{
    double pi = 3.141592653589793;
    double T = 50;
    std::array<double, 3> pose_cur;
    getRobotPose(index, pose_cur);
    double x_robot = pose_cur[0];
    double y_robot = pose_cur[1];
    double theta_robot = pose_cur[2];

    // ux = (x0 - x_robot)*(x0 - x_robot)*(x0 - x_robot) / T;
    // uy = (y0 - y_robot)* (y0 - y_robot)* (y0 - y_robot)/ T;
    ux = (x0 - x_robot) / T;
    uy = (y0 - y_robot) / T;
}

/*获得机器人相对位置矩阵*/
void SwarmRobot::GetGxGyGd(Eigen::MatrixXd &Gx, Eigen::MatrixXd &Gy, Eigen::MatrixXd &Gd)
{
    std::array<double, 3> pose_curi;
    std::array<double, 3> pose_curj;

    for (int i = 0; i < this->robot_num; i++)
    {
        getRobotPose(i, pose_curi);
        for (int j = 0; j < this->robot_num; j++)
        {
            getRobotPose(j, pose_curj);
            Gx(i, j) = pose_curi[0] - pose_curj[0];
            Gy(i, j) = pose_curi[1] - pose_curj[1];
            Gd(i, j) = std::sqrt(Gx(i, j) * Gx(i, j) + Gy(i, j) * Gy(i, j));
        }
    }
    /*dubug*/
    // std::cout << "Gx =" << endl;
    // std::cout << Gx << endl;
    // std::cout << "********************************************************" << endl;
    // std::cout << "Gy=" << endl;
    // std::cout << Gy << endl;
    // std::cout << "********************************************************" << endl;
    // std::cout << "Gd=" << endl;
    // std::cout << Gd << endl;
    // std::cout << "********************************************************" << endl;
}

/*通过相对位置计算速度*/
void SwarmRobot::GetVelocity(Eigen::MatrixXd Gd0, Eigen::MatrixXd Gd, double *ux, double *uy)
{
    std::array<double, 3> pose_curi;
    std::array<double, 3> pose_curj;
    double d = 0;
    Eigen::MatrixXd gd(this->robot_num, this->robot_num);
    gd = (Gd - Gd0);
    // std::cout << "gd =" << endl;
    // std::cout << gd << endl;
    // std::cout << "********************************************************" << endl;
    // std::cout << "Gd0=" << endl;
    // std::cout << Gd0 << endl;
    // std::cout << "********************************************************" << endl;
    // std::cout << "Gd=" << endl;
    // std::cout << Gd << endl;
    // std::cout << "********************************************************" << endl;

    /*获得方向向量,一号车不动,二号车与一号车距离固定,其它车以这两两车为目标计算速度*/
    ux[0] = 0;
    uy[0] = 0;
    double direction[2] = {0};
    double direction0[2] = {0};
    double direction1[2] = {0};
    /*初始化速度*/
    for (int i = 0; i < this->robot_num; i++)
    {
        std::cout << "ux" << i << "=" << ux[i] << endl;
        std::cout << "uy" << i << "=" << uy[i] << endl;
        ux[i] = 0;
        uy[i] = 0;
    }
    /*获得车的速度*/
    /*二号车速度计算*/
    for (int i = 1; i < 2; i++)
    {
        getRobotPose(i, pose_curi);
        for (int j = 0; j < 2; j++)
        {
            getRobotPose(j, pose_curj);
            /*计算面对车1(或2)的方向向量*/
            direction[0] = pose_curj[0] - pose_curi[0];
            direction[1] = pose_curj[1] - pose_curi[1];
            d = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1]);
            if (d != 0)
            {
                direction0[0] = direction[0] / d;
                direction0[1] = direction[1] / d;
            }
            else
            {
                direction0[0] = 0;
                direction0[1] = 0;
            }
            ux[i] += gd(i, j) * direction0[0];
            uy[i] += gd(i, j) * direction0[1];
        }
    }
    /*3和之后的车的速度计算*/
    for (int i = 2; i < this->robot_num; i++)
    {
        getRobotPose(i, pose_curi);
        for (int j = 0; j < 3; j++)
        {
            getRobotPose(j, pose_curj);
            /*计算面对车1(或2)的方向向量*/
            direction[0] = pose_curj[0] - pose_curi[0];
            direction[1] = pose_curj[1] - pose_curi[1];
            d = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1]);
            if (d != 0)
            {
                direction0[0] = direction[0] / d;
                direction0[1] = direction[1] / d;
            }
            else
            {
                direction0[0] = 0;
                direction0[1] = 0;
            }
            ux[i] += gd(i, j) * direction0[0];
            uy[i] += gd(i, j) * direction0[1];
        }
    }
}