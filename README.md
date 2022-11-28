# Lab_8

## Introduction

The purpose of this experiment is primarily to “calibrate” the camera. In order to use the camera to measure the position of objects and to use that information as feedback for the control of the arm, it is necessary to know the pose of the camera in the World coordinate system. That pose is determined by 6 parameters which must be measured. As with many complicated measurements, it is unreasonable to believe that a single measurement is accurate enough. The standard approach is to repeat the measurements a reasonable number of times; determine a least-squares fit for the parameters; then check the fit by means of a separate set of measurements. If the result of the test is not good enough, make more (ideally more accurate) measurements.

Once you have calibrated the camera, it is interesting to check the accuracy of the largest square you found for the end effector in the previous lab. You only specified the corners. MoveIt determined the path and the trajectory. If you repeat them and observe the result with the camera, you can compare the path and trajectory as measured by the camera with that computed from the joint angles.

The first thing you have to do is make the first set of measurements for the calibration and record them. We supply the least squares code. Be sure to collect measurements from a broad range of positions, including some extreme ones.

## Steps for recording calibration points

1. Run the following command to download files for this lab into your folder:

```console
git clone https://github.com/ENRE467/Lab_8.git
```

2. Navigate to `Lab_8/src` directory using the `cd` command.

3. Run the following command so that you can see the GUI applications from docker container in the screen of the host pc:

```console
xhost +local:docker
```

4. Run the following command to run the docker container:

```console
docker run -it --rm --name UR3Container --net=host --pid=host --privileged --env="DISPLAY=$DISPLAY" --volume="$PWD:/home/${USER}/workspace/src" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/dev:/dev:rw" --ulimit rtprio=99 --ulimit rttime=-1 ur3e_image:latest
```

5. Run the following commands:

```console
catkin build
```
```console
source ~/workspace/devel/setup.bash
```

6. Using tmux, split the terminal window into multiple terminals.

7. Start up the real ur3e robot using the tablet and run the following commands in order in different terminals:

```console
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.77.22 kinematics_config:=$(rospack find ur_calibration)/calib/ur3e_calib.yaml z_height:=0.766
```

```console
roslaunch ur_robot_driver example_rviz.launch
```

8. Now run the following command in a different terminal to start the camera and the ArUco tag tracking functionality:

```console
roslaunch camera_calib_pkg extrinsic_calibration.launch aruco_tracker:=true show_output:=true
```

9. In a different terminal, run the following command to start the code for recording calibration data:

```console
roslaunch camera_calib_pkg aruco_tf.launch num_poses:=15
```

10. Move the UR3e arm’s end effector to at least 15 different poses and press enter to record that pose for calibration. Make sure that the aruco tag is in the view of the camera. Once you have recorded the poses, the calibration data will be saved in `camera_calib_pkg/calibration/camera/logitech_extrinsics.json` file.

11. At this point the calibration is tentatively complete. We do not know how accurate it is. In order to determine the accuracy you need to a move the end effector to a different, but also representative, set of 15 points. In order to evaluate the accuracy, you need to perform Step 12.

12. First you have to complete the `verify_calibration()` function in `aruco_tf.cpp`. You need to compute the errors between the calibrated camera-based poses of the new points and the corresponding angle-encoder based poses. Then, compute the vector of sample means and the sample covariance matrix. You may use the `Eigen` library for vector and matrix operations.

13. Stop the program in the terminal in which you did Step 9 and run the following command to load your saved calibration and verify it:

```console
roslaunch camera_calib_pkg aruco_tf.launch load_calibration:=true verify_calibration:=true num_poses:=15
```