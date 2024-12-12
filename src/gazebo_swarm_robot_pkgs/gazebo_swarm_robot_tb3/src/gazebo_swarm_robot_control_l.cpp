#include <gazebo_swarm_robot_control.h>// 包含自定义的头文件
#include <time.h>

/* 主函数 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_robot_control_l"); // 初始化ROS节点

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


    /* 收敛标志 */
    bool ang_conv = false; // 角度收敛标志
    bool pos_conv = false; // 位置收敛标志

    /* While 循环 */
    while(! ang_conv || ! pos_conv) { // 当未达到收敛条件时执行以下代码
        swarm_robot.getRobotPose(current_robot_pose); // 获取机器人姿态信息

        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cur_x(i) = current_robot_pose[i][0]; // 提取x坐标信息
            cur_y(i) = current_robot_pose[i][1]; // 提取y坐标信息
            cur_theta(i) = current_robot_pose[i][2]; // 提取角度信息
        }

        /* 判断是否达到收敛条件 */
        del_theta = -lap * cur_theta; // 计算角度的变化, Laplace矩阵乘原始角度
        ang_conv = true; // 假设已经达到收敛条件
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            if(std::fabs(del_theta(i)) > conv_th) {
                ang_conv = false; // 如果任何一个角度的变化大于阈值，则认为未收敛
            }       
        }
        del_x = -lap * cur_x; // 计算需要的x的变化
        pos_conv = true; // 假设已经达到收敛条件
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            if(std::fabs(del_x(i)) > conv_x) {
                pos_conv = false; // 如果任何一个x坐标的变化大于阈值，则认为未收敛
            }
        }

        /* 移动群体机器人 */
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            double v = del_x(i) * k_v; // 计算线速度
            v = swarm_robot.checkVel(v, MAX_V, MIN_V); // 限制线速度的范围
            double w = del_theta(i) * k_w; // 计算角速度
            w = swarm_robot.checkVel(w, MAX_W, MIN_W); // 限制角速度的范围
            swarm_robot.moveRobot(i, v, w); // 控制机器人的运动，只控制角速度
        }


        /* 等待一段时间以进行机器人移动 */
        ros::Duration(0.05).sleep(); // 暂停程序执行0.05秒，等待机器人移动
    }
    
    /* 停止所有机器人的运动 */
    swarm_robot.stopRobot(); // 调用停止机器人运动的方法

    ROS_INFO_STREAM("Succeed!???"); // 输出成功消息
    return 0; // 返回0，表示程序正常结束
}