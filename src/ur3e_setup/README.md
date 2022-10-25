# Steps to follow for real UR3e Arm

### Robot Preparation
- Make sure External Control URCaps is running
- UR Driver should say: `Robot connected to reverse interface. Ready to receive control commands.`
---
### ROS Preparation
```bash
cd ~/workspace
catkin build
source devel/setup.bash # Once per terminal instance
```
---
### Launch Sequence
```bash
# Initiate UR driver communication with real robot
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.77.21 kinematics_config:=/home/user/workspace/src/ur3e2_calib.yaml z_height:=0.8

# Start MoveIt for UR3e
roslaunch ur3e_moveit_config ur3e_moveit_planning_execution.launch

# Start RViz
roslaunch ur3e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur3e_moveit_config)/launch/moveit.rviz
```