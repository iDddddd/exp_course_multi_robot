/*
 * Date: 2024-9-25
 * Description: the basic function
 */

#include <gazebo_swarm_robot_control.h>

using namespace std;
using namespace chrono;

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

bool SwarmRobot::getRobotPose(int index, std::array<double, 3>& pose_cur)
{
  tf::StampedTransform transform;
  std::string robot_frame = "robot_" + std::to_string(this->swarm_robot_id[index]) + "/base_footprint";
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

  // ROS_INFO_STREAM("Get pose of robot_" << swarm_robot_id[index] << " is: x=" << pose_cur[0] << " y=" << pose_cur[1]
  // << " theta=" << pose_cur[2]);
  return true;
}

bool SwarmRobot::getRobotPose(std::vector<std::array<double, 3>>& current_robot_pose)
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

void SwarmRobot::getRobotSpeed(int index, std::array<double, 2>& speed)
{
  speed[0] = this->speed[index][0];
  speed[1] = this->speed[index][1];
}

void SwarmRobot::getRobotSpeed(std::vector<std::array<double, 2>>& swarm_speed)
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

bool SwarmRobot::moveRobot(std::vector<std::array<double, 2>>& speed)
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
    this->speed[i][0] = 0.0;
    this->speed[i][1] = 0.0;
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

void SwarmRobot::U2VW(int index, double ux, double uy, double& v, double& w)
{
  const double pi = 3.141592653589793;
  // 获取当前机器人的姿态
  std::array<double, 3> pose_cur;
  getRobotPose(index, pose_cur);
  double theta_robot = pose_cur[2];

  double v0 = std::sqrt(ux * ux + uy * uy);
  double theta_v = std::atan2(uy, ux);  // 直接计算速度方向

  // 限定角度范围 [-pi, pi]
  double angle = theta_v - theta_robot;
  while (angle > pi)
    angle -= 2 * pi;
  while (angle < -pi)
    angle += 2 * pi;

  double W = 1;  // 最大角速度参考值
  double V = 1;  // 速度方向

  // 调整角度和速度方向
  if (angle > pi / 2)
  {
    angle -= pi;
    V = -1;
  }
  else if (angle < -pi / 2)
  {
    angle += pi;
    V = -1;
  }

  // 计算角速度和线速度
  w = W * (angle / std::abs(angle)) * (std::exp(std::abs(angle)) - 1);
  v = V * v0 * std::exp(-std::abs(angle));

  // 限制线速度幅值
  if (v > 0.5)
    v = 0.5;
  else if (v < -0.5)
    v = -0.5;
  if (v == 0)
    w = 0;
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

double SwarmRobot::getSign(double value)
{
  if (value > 0)
  {
    return 1.0;  // 正数
  }
  else if (value < 0)
  {
    return -1.0;  // 负数
  }
  else
  {
    return 0.0;  // 零
  }
}
void SwarmRobot::reallocation(Eigen::VectorXd& tar_x, Eigen::VectorXd& tar_y)
{
  //获取当前机器人的位置
  std::vector<std::array<double, 3>> current_robot_pose(this->robot_num);
  this->getRobotPose(current_robot_pose);
  double ave_x = 0;
  double ave_y = 0;

  // 计算当前机器人的位置
  Eigen::VectorXd cur_x(this->robot_num);
  Eigen::VectorXd cur_y(this->robot_num);
  for (int i = 0; i < this->robot_num; i++)
  {
    cur_x(i) = current_robot_pose[i][0];
    cur_y(i) = current_robot_pose[i][1];
    ave_x += cur_x(i);
    ave_y += cur_y(i);
  }
  ave_x /= this->robot_num;
  ave_y /= this->robot_num;

  for(int i = 0; i < this->robot_num; i++){
    cur_x(i) -= ave_x;
    cur_y(i) -= ave_y;
  }

  //存储当前目标位置
  Eigen::VectorXd tar_x_tmp = tar_x;
  Eigen::VectorXd tar_y_tmp = tar_y;
  // 初始化分配结果和目标是否已被访问
  std::vector<int> allocation(this->robot_num, -1);
  std::vector<double> min_distances(this->robot_num, std::numeric_limits<double>::max());
  std::vector<int> target_IS(this->robot_num, -1);
  bool update;
  do
  {
    update = false;

    for (int i = 0; i < this->robot_num; ++i)
    {
      if (allocation[i] != -1)
        continue;
      double min_distance = std::numeric_limits<double>::max();
      int closest_target = -1;
      for (int j = 0; j < this->robot_num; ++j)
      {
        double dx = cur_x[i] - tar_x[j];
        double dy = cur_y[i] - tar_y[j];
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < min_distance)
        {
          min_distance = distance;
          closest_target = j;
        }
      }
      if (closest_target != -1)
      {
        if (target_IS[closest_target] != -1)
        {
          //   cout << i << " " << min_distance << endl;
          //   cout << "target_IS: " << target_IS[closest_target] << " ";
          cout << min_distances[target_IS[closest_target]] << endl;
          if (min_distance < min_distances[target_IS[closest_target]])
          {
            min_distances[target_IS[closest_target]] = std::numeric_limits<double>::max();
            allocation[target_IS[closest_target]] = -1;
            update = true;
            // cout << "update" << endl;
          }
          else
          {
            min_distance = std::numeric_limits<double>::max();
            for (int j = 0; j < this->robot_num; ++j)
            {
              if (target_IS[j] != -1)
                continue;
              double dx = cur_x[i] - tar_x[j];
              double dy = cur_y[i] - tar_y[j];
              double distance = std::sqrt(dx * dx + dy * dy);
              if (distance < min_distance)
              {
                min_distance = distance;
                closest_target = j;
              }
            }
            // cout << "next: " << closest_target << endl;
          }
        }
      }
      target_IS[closest_target] = i;
      min_distances[i] = min_distance;
      allocation[i] = closest_target;
    }

    cout << "min_distances: ";
    for (int i = 0; i < this->robot_num; ++i)
    {
      cout << min_distances[i] << " ";
    }
    cout << endl;
  } while (update);

  // 重新分配目标位置
  for (int i = 0; i < this->robot_num; ++i)
  {
    tar_x[i] = tar_x_tmp[allocation[i]];
    tar_y[i] = tar_y_tmp[allocation[i]];
  }
}

void SwarmRobot::pos_control(const Eigen::VectorXd tar_x, const Eigen::VectorXd tar_y, const Eigen::MatrixXd lap,
                             bool is_absolute)
{
  /* 收敛阈值 */
  double conv_th = 0.05;  // 角度的阈值，单位弧度
  double conv_x = 0.05;   // x的阈值，单位m
  double conv_y = 0.05;

  //获取当前机器人的位置
  std::vector<std::array<double, 3>> current_robot_pose(this->robot_num);

  /* 存储机器人当前位姿和与其他机器人位姿差的 Eigen 对象 */
  Eigen::VectorXd cur_x(this->robot_num);
  Eigen::VectorXd cur_y(this->robot_num);
  Eigen::VectorXd cur_theta(this->robot_num);
  Eigen::VectorXd del_x(this->robot_num);
  Eigen::VectorXd del_y(this->robot_num);
  Eigen::VectorXd del_theta(this->robot_num);

  bool is_conv = false;

  while (!is_conv)
  {  // 当未达到收敛条件时执行以下代码

    getRobotPose(current_robot_pose);  // 获取机器人姿态信息
    for (int i = 0; i < this->robot_num; i++)
    {
      cur_x(i) = current_robot_pose[i][0];      // 提取位置信息
      cur_y(i) = current_robot_pose[i][1];      // 提取位置信息
      cur_theta(i) = current_robot_pose[i][2];  // 提取角度信息
    }
    /* 判断是否达到收敛条件 */
    if (is_absolute)
    {
      //绝对位置
      del_x = -(cur_x - tar_x);  // 计算需要的x的变化
      del_y = -(cur_y - tar_y);  // 计算需要的y的变化
    }
    else
    {
      //相对位置
      del_x = -lap * (cur_x - tar_x);  // 计算需要的x的变化
      del_y = -lap * (cur_y - tar_y);  // 计算需要的y的变化
    }
    is_conv = true;  // 假设已经达到收敛条件

    for (int i = 0; i < this->robot_num; i++)
    {
      // cout << "del_x:" << del_x(i) << endl << "del_y:" << del_y(i) << endl;
      if (std::abs(del_x(i)) > conv_x || std::abs(del_y(i)) > conv_y)
      {
        is_conv = false;  // 如果任何一个坐标的变化大于阈值，则认为未收敛
      }
    }

    for (int i = 0; i < this->robot_num; i++)
    {
      double v = 0;
      double w = 0;
      double ux = del_x(i) * 0.2;
      double uy = del_y(i) * 0.2;
      U2VW(i, ux, uy, v, w);
      moveRobot(i, v, w);
    }

    /* 等待一段时间以进行机器人移动 */
    ros::Duration(0.05).sleep();  // 暂停程序执行0.05秒，等待机器人移动
  }
  cout<<"success get target position"<<endl;
  stopRobot();
}

void SwarmRobot::speed_control(const double tar_x_speed, const double tar_y_speed, const Eigen::MatrixXd lap, int time)
{
  // // 机器人当前速度
  // std::vector<std::array<double, 2>> current_robot_speed(this->robot_num);

  // /* 存储机器人当前速度和与其他机器人速度差的 Eigen 对象 */
  // Eigen::VectorXd cur_x_speed(this->robot_num);
  // Eigen::VectorXd cur_y_speed(this->robot_num);
  Eigen::VectorXd del_x(this->robot_num);
  Eigen::VectorXd del_y(this->robot_num);
  Eigen::VectorXd tar_angle(this->robot_num);

  for (int i = 0; i < this->robot_num; i++)
  {
    del_x(i) = tar_x_speed;
    del_y(i) = tar_y_speed;
    tar_angle(i) = atan2(tar_y_speed, tar_x_speed);
  }
  //先旋转到目标角度
  //angle_control(tar_angle, lap, true);

  auto start = system_clock::now();
  auto end = system_clock::now();
  auto duration = duration_cast<milliseconds>(end - start);

  /* 运行直到各个机器人速度达到目标速度 */
  while (duration < milliseconds(time))
  {
    // // 获取机器人当前速度
    // getRobotSpeed(current_robot_speed);

    // for (int i = 0; i < this->robot_num; i++)
    // {
    //   cur_x_speed(i) = current_robot_speed[i][0];  // 提取位置信息
    //   cur_y_speed(i) = current_robot_speed[i][1];  // 提取位置信息
    // }

    // 移动机器人
    moveRobotsbyU(del_x, del_y);

    // 等待一段时间
    ros::Duration(0.05).sleep();

    //到达设定时间跳出循环
    end = system_clock::now();
    duration = duration_cast<milliseconds>(end - start);
  }
  
  cout << "success get target speed" << time << " ms" << endl;
  stopRobot();
}

void SwarmRobot::angle_control(const Eigen::VectorXd tar_angle, const Eigen::MatrixXd lap, bool is_absolute)
{
  // 机器人当前位姿
  std::vector<std::array<double, 3>> current_robot_pose(this->robot_num);

  Eigen::VectorXd cur_theta(this->robot_num);
  Eigen::VectorXd del_theta(this->robot_num);

  double conv_th = 0.05;  // 角度跟踪的阈值(小于该值认为到达)(rad)

  /* 运行直到各个机器人角度相同 */
  bool is_conv = false;  // 是否到达
  while (!is_conv)
  {
    // 获取机器人当前位姿
    getRobotPose(current_robot_pose);
    // 提取角度信息, 赋值给 cur_theta
    for (int i = 0; i < this->robot_num; i++)
    {
      cur_theta(i) = current_robot_pose[i][2];
    }
    // 判断是否到达
    if (is_absolute)
      del_theta = -(cur_theta - tar_angle);
    else
      del_theta = -lap * (cur_theta - tar_angle);

    is_conv = true;

    for (int i = 0; i < this->robot_num; i++)
    {
      if (std::abs(del_theta(i)) > conv_th)
        is_conv = false;
    }
    // 控制机器人运动
    for (int i = 0; i < this->robot_num; i++)
    {
      double w = del_theta(i) * 0.5;
      w = checkVel(w, MAX_W, MIN_W);
      moveRobot(i, 0.0, w);
    }

    // 等待一段时间
    ros::Duration(0.05).sleep();
  }
  cout<<"success get target angle"<<endl;
  // 停止所有机器人
  stopRobot();
}