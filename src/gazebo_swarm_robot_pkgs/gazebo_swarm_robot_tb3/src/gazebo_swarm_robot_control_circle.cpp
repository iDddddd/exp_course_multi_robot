#include <gazebo_swarm_robot_control.h> // 包含自定义的头文件
#include <math.h>
#include <Eigen/Dense> // 包含Eigen库
#include <Eigen/Core>

#define pi 3.1415926

/* 主函数 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_robot_control_circle"); 

    // 机器人的id
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5};

    // 创建 SwarmRobot 对象
    SwarmRobot swarm_robot(swarm_robot_id);

   // Laplace matrix
    Eigen::MatrixXd lap(swarm_robot.robot_num, swarm_robot.robot_num);
    lap << 4, -1, -1, -1, -1,
        -1, 4, -1, -1, -1,
        -1, -1, 4, -1, -1,
        -1, -1, -1, 4, -1,
        -1, -1, -1, -1, 4;

    /* 收敛阈值 */
    double conv_th = 0.05;  // 角度的阈值，单位弧度
    double conv_x = 0.05;  // x的阈值，单位m
    double conv_y = 0.05;


   /* Velocity scale and threshold */
    double MAX_W = 1;    // 最大角速度 (rad/s)
    double MIN_W = 0.05; // 最小角速度 (rad/s)
    double MAX_V = 0.2;  // 最大线速度 (m/s)
    double MIN_V = 0.01; // 最小线速度 (m/s)
    double k_w = 0.1; // 期望角速度 = k_w * del_theta, del_theta 为某机器人与其他机器人的角度差
    double k_v = 0.1;

     /* 存储机器人当前位姿和与其他机器人位姿差的 Eigen 对象 */
    Eigen::VectorXd cur_x(swarm_robot.robot_num);
    Eigen::VectorXd cur_y(swarm_robot.robot_num);
    Eigen::VectorXd cur_theta(swarm_robot.robot_num);
    Eigen::VectorXd del_x(swarm_robot.robot_num);
    Eigen::VectorXd del_y(swarm_robot.robot_num);
    Eigen::VectorXd del_theta(swarm_robot.robot_num);
    
    /* 首先获取群体机器人的姿态信息 */
    std::vector<std::array<double, 3>> current_robot_pose(swarm_robot.robot_num);

    swarm_robot.getRobotPose(current_robot_pose); // 获取机器人姿态信息

    /*编队*/
    Eigen::VectorXd tar_x(swarm_robot.robot_num);
    Eigen::VectorXd tar_y(swarm_robot.robot_num);


    //圆形
    tar_x << cos(pi/6),cos(pi/3) , -cos(pi/6), 0, -cos(pi/3);
    tar_y <<  sin(-pi/6),sin(pi/3), sin(-pi/6), -1, sin(pi/3);
    
    double target_radius = 1; // 目标半径
    tar_x *= target_radius;
    tar_y *= target_radius;

    //直线
    // tar_x << -2, -1, 0, 1, 2;
    // tar_y << -1, -1, -1, -1, -1;


    bool is_conv = false;

    while(! is_conv) { // 当未达到收敛条件时执行以下代码
        swarm_robot.getRobotPose(current_robot_pose); // 获取机器人姿态信息
        for(int i = 0; i < swarm_robot.robot_num; i++) {
            cur_x(i) = current_robot_pose[i][0]; // 提取位置信息
            cur_y(i) = current_robot_pose[i][1]; // 提取位置信息
            cur_theta(i) = current_robot_pose[i][2]; // 提取角度信息
        }
        /* 判断是否达到收敛条件 */
        del_x = -lap * (cur_x + tar_x); // 计算需要的x的变化
        del_y = -lap * (cur_y + tar_y); // 计算需要的y的变化
        is_conv = true; // 假设已经达到收敛条件

        for(int i = 0; i < swarm_robot.robot_num; i++) {
            cout << del_x(i) << " " << del_y(i) << endl;
            if ( (std::fabs(del_x(i)) > conv_x) or (std::fabs(del_y(i)) > conv_y) ) {
                is_conv = false; // 如果任何一个坐标的变化大于阈值，则认为未收敛
            }       
        }

        for (int i = 0; i < swarm_robot.robot_num; i++){
            double v = 0;
            double w = 0;
            double ux = del_x(i) * 0.2;
            double uy = del_y(i) * 0.2;
            swarm_robot.U2VW(i, ux, uy, v, w);
            swarm_robot.moveRobot(i, v, w);
        }

        /* 等待一段时间以进行机器人移动 */
        ros::Duration(0.05).sleep(); // 暂停程序执行0.05秒，等待机器人移动

    }

    swarm_robot.stopRobot();

    ROS_INFO_STREAM("Succeed!"); // 输出成功消息
    return 0; // 返回0，表示程序正常结束
}
    