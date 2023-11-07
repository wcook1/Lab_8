# Steps to follow for real UR3e Arm

### Robot Preparation
- Make sure External Control URCaps is running
- UR Driver should say: `Robot connected to reverse interface. Ready to receive control commands.`
---
### ROS Preparation
```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
tmux new-session \; \split-window -v \; \split-window -h \; \select-pane -t 1 \; \split-window -h
```
---
### Launch Sequence
Initiate UR driver communication with real robot
```bash
roslaunch ur3e_setup ur3e_bringup_mrc.launch robot_ip:=192.168.77.22 kinematics_config:=$(rospack find ur3e_setup)/config/ur3e_calib.yaml z_height:=0.77
```

# Start MoveIt for UR3e. It also launches Rviz
```bash
roslaunch ur3e_moveit_mrc ur3e_moveit.launch
```
