/*
 * Date: 2024-9-25
 * Description: the basic function
 */

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Geometry>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <cmath>

using std::cout;
using std::endl;

class SwarmRobot
{

public:
    /**
     * @brief 构造函数
     * @param swarm_robot_id_ 机器人的id
     */
    SwarmRobot(std::vector<int> swarm_robot_id_);
    ~SwarmRobot();

    // 机器人的id列表
    std::vector<int> swarm_robot_id;

    // 机器人数量
    int robot_num;

    /**
     * @brief 获取第 `index` 个机器人的位姿
     * @param index 机器人的序号
     * @param pose_cur 获取到的机器人的位姿, 包含 x, y, yaw
     * @return 获取成功返回 true, 否则返回 false
     */
    bool getRobotPose(int index, std::array<double, 3> &pose_cur);

    /**
     * @brief 获取所有机器人的位姿
     * @param swarm_pose_cur 获取到的所有机器人的位姿
     * @return 获取成功返回 true, 否则返回 false
     */
    bool getRobotPose(std::vector<std::array<double, 3>> &swarm_pose_cur);

    /**
     * @brief 获取第 `index` 个机器人的速度
     * @param index 机器人的序号
     * @param swarm_speed 获取到的机器人的速度, 包含 v 和 w
     */
    void getRobotSpeed(int index, std::array<double, 2> &speed);

    /**
     * @brief 获取所有机器人的速度
     * @param swarm_speed 获取到的所有机器人的速度, 每一项包含 v 和 w
     */
    void getRobotSpeed(std::vector<std::array<double, 2>> &swarm_speed);

    /**
     * @brief 移动第 `index` 个机器人
     * @param index 机器人的序号
     * @param v 机器人的线速度
     * @param w 机器人的角速度
     */
    bool moveRobot(int index, double v, double w);

    /**
     * @brief 移动所有机器人
     * @param speed 机器人的速度列表, 每一项包含 v 和 w
     * @return 移动成功返回 true, 否则返回 false
     */
    bool moveRobot(std::vector<std::array<double, 2>> &speed);

    /**
     * @brief 停止第 `index` 个机器人的移动
     * @param index 机器人的序号
     */
    bool stopRobot(int index);

    /**
     * @brief 停止所有机器人的移动
     */
    bool stopRobot();

    /**
     * @brief 将机器人的速度限制在最大值和最小值之间
     */
    double checkVel(double v, double max_v, double min_v);

    /**
     * @brief 将机器人的期望速度转换为线速度和角速度
     * @param index 机器人的序号
     * @param ux 机器人的期望速度x
     * @param uy 机器人的期望速度y
     * @param v 获取到的机器人期望线速度
     * @param w 获取到的机器人期望角速度
     */
    void U2VW(int index, double ux, double uy, double &v, double &w);

    /**
     * @brief 移动第 `index` 个机器人
     * @param index 机器人的序号
     * @param ux 机器人的期望速度x
     * @param uy 机器人的期望速度y
     */
    void moveRobotbyU(int index, double ux, double uy);

    /**
     * @brief 移动所有机器人
     * @param del_x 机器人的期望速度x
     * @param del_y 机器人的期望速度y
     */
    void moveRobotsbyU(Eigen::VectorXd del_x, Eigen::VectorXd del_y);

    void Formation(Eigen::VectorXd needed_x, Eigen::VectorXd needed_y, Eigen::MatrixXd lap, double conv_x, double conv_y);

    void MoveFormation(Eigen::MatrixXd Gd, Eigen::MatrixXd lap, double v_x, double v_y);

    void ChangeFormationDirection(double target_direction);

    void ComeDot(int index, double x0, double y0, double &ux, double &uy);

    void GetGxGyGd(Eigen::MatrixXd &Gx, Eigen::MatrixXd &Gy, Eigen::MatrixXd &Gd);

    void GetVelocity(Eigen::MatrixXd Gd0, Eigen::MatrixXd Gd, double *ux, double *uy);

private:
    // ROS tf listener
    tf::TransformListener tf_listener;
    // 机器人移动命令发布者
    std::vector<ros::Publisher> cmd_vel_pub;

    // ROS Nodehandle
    ros::NodeHandle nh_;

    // 速度记录, 每一项包含 v 和 w
    std::vector<std::array<double, 2>> speed;

    const double MAX_W = 1;    // 最大角速度(rad/s)
    const double MIN_W = 0.05; // 最小角速度(rad/s)
    const double MAX_V = 0.2;  // 最大线性速度(m/s)
    const double MIN_V = 0.01; // 最小线性速度(m/s)

    /**
     * @brief 获取一个数的符号
     */
    double getSign(double value);
};