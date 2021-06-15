# Smarteye_Moveit_Calibration
ROS1 node for getting image from Smarteye camera and send the image with camera info to moveit calibration tool to get hand-eye-matrix

## Pre-requests
 - ROS Noetic or Melodic
 - Moveit
 - Moveit-Calibration:
   
   Refer to https://ros-planning.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html

## Install
1. Compile the source package under workspace by "catkin_make"
2. Source "devel/setup.bash" of workspace
3. Export smarteye lib path:
`````
export LD_LIBRARY_PATH=~/workspace/src/libSmarteye/src/lib/SmartEyeAPI:$LD_LIBRARY_PATH
`````

## Usage
1. launch robot driver and moveit with Rviz. In Rviz, add new panel named "HandEyeCalibration"
2. Specify your Smarteye camera parameter in [smarteye.launch](libSmarteye/launch/smarteye.launch), such as "min_length", "max_length", "min_width", "max_width", "min_depth", "max_depth", "exp_time_2D".
3. Specify your camera infos in [RosApi.cpp](libSmarteye/src/lib/RosApi.cpp), such as Smarteye Left Camera's "cam_info.D", "cam_info.K", "cam_info.R", "cam_info.P". Remember to "catkin_make" again after you changed the source file.
4  launch smarteye node:
`````
roslaunch smarteye smarteye.launch
`````
5. run trigger node:
`````
rosrun smarteye trigger_node.py
`````
6. follow the official instructions of Moveit-Calibration at https://ros-planning.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html to continue.