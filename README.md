## 安装

安装ros2及以下软件包：

```bash
sudo apt-get install ros-humble-ros2-control
sudo apt-get install ros-humble-ros2-controllers
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-gazebo-ros2-control
```

编译源码安装

```bash
# create a ros2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/

# clone mir_robot into the ros2 workspace
git clone https://github.com/Spartan-Velanjeri/UR-MiR-mobile-manipulator.git src/mir_robot

# use vcs to fetch linked repos
# $ sudo apt install python3-vcstool
vcs import < src/mir_robot/mir_robot/ros2.repos src --recursive

# use rosdep to install all dependencies (including ROS itself)
sudo apt update
sudo apt install -y python3-rosdep
rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# build all packages in the workspace
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build

.bashrc中添加
source ~/ros2_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/sea/.gazebo/models
LC_NUMERIC=en_US.UTF-8
. /usr/share/gazebo/setup.sh
```

## 建图

```bash
# 打开gazebo以及rviz
ros2 launch mir_gazebo mobile_manipulator.launch.py world:=office
# world可选择office,cafe,

# 建图
ros2 launch mir_navigation mapping.py use_sim_time:=true slam_params_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/config/mir_mapping_async_sim.yaml

# 保存地图
ros2 run nav2_map_server map_saver_cli -f <path>
```

## Gazebo导航

```bash
# gazebo+rviz
ros2 launch mir_gazebo mobile_manipulator.launch.py world:=office rviz_config_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/rviz/mir_nav.rviz
# 加载已存在的地图
ros2 launch mir_navigation amcl.py use_sim_time:=true map:=$HOME/ros2_ws/install/mir_gazebo/share/mir_gazebo/maps/office.yaml
# nav导航服务
ros2 launch mir_navigation navigation.py use_sim_time:=true
# 启动节点
ros2 run my_nav_pkg nav_cra
```

## TODO:Gazebo+MoveIt2

```
### gazebo:
ros2 launch mir_gazebo mobile_manipulator.launch.py world:=good
ros2 launch mir_gazebo mobile_manipulator.launch.py world:=good rviz_config_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/rviz/mir_nav.rviz
### MoveIt2:
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true prefix:=ur_ use_fake_hardware:=true use_sim_time:=true

### Gazebo Aruco model path (Also put in the src/mir_robot/mir_gazebo/worlds/include folder)
/install/mir_gazebo/share/mir_gazebo/worlds/include/marker_0001

### Aruco tag recognition (Make sure to add camera img to Rviz before running this)
ArUco 标签识别
ros2 launch ros2_aruco aruco_recognition.launch.py
功能：
启动 ArUco 标签识别节点，通过摄像头检测环境中的 ArUco 标签（需提前在 RViz 中添加摄像头图像话题）。
输出：
发布标签的 ID 和位姿（/aruco_markers 话题）。

### Aruco pose to nav2 goal
ArUco 位姿转导航目标
ros2 run ros2_aruco aruco_pose_to_nav_goal
功能：
将检测到的 ArUco 标签位姿 转换为 Nav2 导航目标，控制 MiR 底盘移动至标签位置。
依赖：
需先启动 aruco_recognition 和 Nav2 导航栈。

### Aruco pose to manipulator goal
 ArUco 位姿转机械臂目标
ros2 run ros2_aruco aruco_pose_to_manipulate
功能：
将 ArUco 标签位姿转换为 机械臂末端执行器的目标位姿，控制 UR5e 抓取或操作标签附近的物体。
依赖：
MoveIt2 和 aruco_recognition 需正常运行。

### Origin pose to nav2 goal
./src/mir_robot/mir_navigation/nav2_test.py 
原点位姿转导航目标

### Tranform aruco pose to ur_base_link
ros2 run ros2_aruco aruco_pose_to_moveit
ArUco 位姿转机械臂基坐标系
功能：
将 ArUco 标签的位姿（相对于摄像头）转换到 UR5e 基坐标系（ur_base_link），为 MoveIt2 提供机械臂可用的目标位姿。
关键点：
依赖 TF 树中 camera_link → ur_base_link 的坐标变换。

### hello_ur_moveit
ros2 launch hello_moveit_ur hello_moveit_ur_launch.py 

```
