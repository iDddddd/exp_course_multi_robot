# 使用 Docker 容器

1. 在任意 linux 系统上安装 docker, 在 ubuntu 中为:
   ```sh
   # Add Docker's official GPG key:
   sudo apt-get update
   sudo apt-get install ca-certificates curl
   sudo install -m 0755 -d /etc/apt/keyrings
   sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
   sudo chmod a+r /etc/apt/keyrings/docker.asc

   # Add the repository to Apt sources:
   echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
      $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   sudo apt-get update

   sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
   ```
2. 拉取 `ros:noetic` 镜像
   ```sh
   docker pull ros:noetic
   ```
3. 在 VSCode 中打开 `mr_ws` 工程(如果工程文件夹名称不一样， 需要修改为 `mr_ws`!!!)
   ```sh
   cd mr_ws
   code .
   ```
4. 在 VSCode 中安装 `Dev Containers` 扩展
5. 按 `Ctrl+Shift+P` 打开命令面板, 搜索并选择 `Dev Containers: Rebuild and Reopen in Containers`
6. 等待完成即可

# 角度,位置一致性协议

## ROS工作空间创建及编译
在 `README.md` 所在的文件夹下启动终端, 在终端运行 `catkin_make` 进行编译(每次改完代码也记得编译一下)
```sh
catkin_make
```
## 添加环境变量
> 使用 `Dev Containers` 打开无需进行

每次新建一个终端运行ros程序时,需要添加一下环境变量
```sh
source devel/setup.bash
```
也可以把这句话加进系统的.bashrc文件里,这样每次打开终端都会自动运行这句语句:
```sh
echo "source /<path to workspace>/devel/setup.bash" >> ~/.bashrc
```
> `/<path to workspace>` 表示工程文件夹所在的路径

## 打开多机器人Gazebo仿真平台
```
roslaunch gazebo_swarm_robot_tb3 gazebo_swarm_robot_5.launch
```

## 运行多移动机器人运动控制程序
**角度一致性**
```sh
rosrun gazebo_swarm_robot_tb3 gazebo_swarm_robot_control_angle
```
或
```sh
rosrun gazebo_swarm_robot_tb3 gazebo_swarm_robot_control_angle.py
```

**速度一致性**
```sh
rosrun gazebo_swarm_robot_tb3 gazebo_swarm_robot_control_u
```
或
```sh
rosrun gazebo_swarm_robot_tb3 gazebo_swarm_robot_control_u.py
```