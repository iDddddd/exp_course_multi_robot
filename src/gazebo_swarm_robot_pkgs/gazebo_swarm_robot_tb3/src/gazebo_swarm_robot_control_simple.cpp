#include <gazebo_swarm_robot_control.h>  // 包含自定义的头文件
#include <math.h>
#include <Eigen/Dense>  // 包含Eigen库
#include <Eigen/Core>

#define pi 3.1415926

/* 主函数 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "swarm_robot_control_simple");  // 初始化ROS节点

  // 机器人的id
  std::vector<int> swarm_robot_id{ 1, 2, 3, 4, 5 };

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
  double conv_x = 0.05;   // x的阈值，单位m
  double conv_y = 0.05;

  /* Velocity scale and threshold */
  double k_w = 0.1;     // 期望角速度 = k_w * del_theta, del_theta 为某机器人与其他机器人的角度差
  double k_v = 0.1;

  /* 存储机器人当前位姿和与其他机器人位姿差的 Eigen 对象 */
  Eigen::VectorXd cur_x(swarm_robot.robot_num);
  Eigen::VectorXd cur_y(swarm_robot.robot_num);
  Eigen::VectorXd cur_theta(swarm_robot.robot_num);
  Eigen::VectorXd del_x(swarm_robot.robot_num);
  Eigen::VectorXd del_y(swarm_robot.robot_num);
  Eigen::VectorXd del_theta(swarm_robot.robot_num);

  /* 首先获取群体机器人的信息 */
  // 机器人当前位姿
  std::vector<std::array<double, 3>> current_robot_pose(swarm_robot.robot_num);
  // 机器人当前速度
  std::vector<std::array<double, 2>> current_robot_speed(swarm_robot.robot_num);

  /*编队*/
  Eigen::VectorXd tar_x(swarm_robot.robot_num);
  Eigen::VectorXd tar_y(swarm_robot.robot_num);

  //圆形
  // tar_x << cos(pi/6),cos(pi/3) , -cos(pi/6), 0, -cos(pi/3);
  // tar_y <<  sin(-pi/6),sin(pi/3), sin(-pi/6), -1, sin(pi/3);

  //三角形
  tar_x << -0.5, -1, 0.5, 0, 1;
  tar_y << 1, 0, 1, 2, 0;

  //   //直线
  //   tar_x << 1, 0, -1, 0, 0;
  //   tar_y << 0,  1, 0, -1, 0;

  swarm_robot.getRobotPose(current_robot_pose);  // 获取机器人姿态信息
  for (int i = 0; i < swarm_robot.robot_num; i++)
  {
    cur_x(i) = current_robot_pose[i][0];      // 提取位置信息
    cur_y(i) = current_robot_pose[i][1];      // 提取位置信息
    cur_theta(i) = current_robot_pose[i][2];  // 提取角度信息
  }

  swarm_robot.reallocation(tar_x, tar_y);

  // 打印当前位置和目标位置
  for (int i = 0; i < swarm_robot.robot_num; i++)
  {
    cout << "cur_x: " << cur_x(i) << " cur_y: " << cur_y(i) << " tar_x: " << tar_x(i) << " tar_y: " << tar_y(i) << endl;
  }

  //按c继续，按q退出
  char c = getchar();
  if (c == 'q')
  {
    return 0;
  }
  else if (c == 'c')
  {
    // do nothing
  }

  swarm_robot.pos_control(tar_x, tar_y, lap, true);

  double tar_x_speed = 0.2;
  double tar_y_speed = 0;

  
  int time = 5*1000;

  swarm_robot.speed_control(tar_x_speed, tar_y_speed,time);

  swarm_robot.stopRobot();

  ROS_INFO_STREAM("Succeed!");  // 输出成功消息
  return 0;                     // 返回0，表示程序正常结束
}
