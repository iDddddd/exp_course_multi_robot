#include <gazebo_swarm_robot_control.h>  // 包含自定义的头文件
#include <math.h>
#include <Eigen/Dense>  // 包含Eigen库
#include <Eigen/Core>

#define pi 3.1415926

/* 主函数 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "swarm_robot_control_move_6");  // 初始化ROS节点

  // 机器人的id
  std::vector<int> swarm_robot_id{ 1, 2, 3, 4, 5 ,6};

  // 创建 SwarmRobot 对象
  SwarmRobot swarm_robot(swarm_robot_id);

  // Laplace matrix
  Eigen::MatrixXd lap(swarm_robot.robot_num, swarm_robot.robot_num);
  lap << 5, -1, -1, -1, -1, -1, 
        -1, 5, -1, -1, -1, -1,
        -1, -1, 5, -1, -1, -1, 
        -1, -1, -1, 5, -1, -1, 
        -1, -1, -1, -1, 5, -1, 
        -1, -1, -1, -1, -1, 5;

 
  /*六边形编队*/
  Eigen::VectorXd tar_x_1(swarm_robot.robot_num);
  Eigen::VectorXd tar_y_1(swarm_robot.robot_num);

  tar_x_1 << -1, 0, 1, 1, 0, -1;
  tar_y_1 << 0.5, 1, 0.5, -0.5, -1, -0.5;

  //*长方形编队*/
  Eigen::VectorXd tar_x_2(swarm_robot.robot_num);
  Eigen::VectorXd tar_y_2(swarm_robot.robot_num);

  tar_x_2 << -1, 0, 1, 1, 0, -1;
  tar_y_2 << 0.25, 0.25, 0.25, -0.25, -0.25, -0.25;
  double tar_x_speed = 0.5;
  double tar_y_speed = 0;

  int time = 1000;

  swarm_robot.reallocation(tar_x_1, tar_y_1);

  swarm_robot.pos_control(tar_x_1, tar_y_1, lap, true);

  time = 1000*6;

  swarm_robot.speed_control(tar_x_speed, tar_y_speed, lap, time);

  swarm_robot.reallocation(tar_x_2, tar_y_2);

  swarm_robot.pos_control(tar_x_2, tar_y_2, lap, false);

  time = 1000*12;

  swarm_robot.speed_control(tar_x_speed, tar_y_speed, lap, time);

  swarm_robot.reallocation(tar_x_1, tar_y_1);

  swarm_robot.pos_control(tar_x_1, tar_y_1, lap, false);

  time = 1000*6;

  swarm_robot.speed_control(tar_x_speed, tar_y_speed, lap, time);

  swarm_robot.stopRobot();

  ROS_INFO_STREAM("Succeed!");  // 输出成功消息
  return 0;                     // 返回0，表示程序正常结束
}
