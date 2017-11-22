OVERVIEW OF FILES

# cc_fabmap_node.cpp
	This is the main program. Settings and parameters can be adjusted here. Instances of ROS nodes spinning, publishing and subscribing to messages are created here.
    Furthermore, helper functions dependent on the CentralStoragevclass are implemented here
    NOTE: The name is confusing and wrong. Should be main.cpp

# CentralStorage.h/.cpp
	Contains the CentralStorage class. Here all important data received from other nodes are stored in variables.

# KeyFrameListenerTraining.h/.cpp
    When creating an instance of this class, it creates a ROS node listening to images.
	Its result is a vocabulary (BOW/visual words), chow-liu tree and the BOW-descriptors of the training data
	This is used for the FABMAP algorithm

# SyncListener.h/.cpp
    When creating an instance of this class, it creates a ROS node listening to .... and makes sure that the message are synchronized (approximate time)

# HelperFcts.h/.cpp
	Static helper functions depending only on standard and thirdparty libraries (not on own classes)



#SETTING UP SYSTEM ON COMPUTER:	
Neccessary libraries:
OPENCV 2.4.9 (http://opencv.org/downloads.html choose version 2.4.9!)
OPENGV (http://laurentkneip.github.io/opengv/page_installation.html)

After installation put...
PCL (http://pointclouds.org/downloads/linux.html)
EIGEN





#RUNNING THE SYSTEM:

Run roscore:
roscore


Run Bag Files (alternatively live with (usb) camera):
cd ~/ROS_HKUST_project/CCFabMap_catkin_ws
./sourceRunCoreBag.sh


Rectify images for each robot:
ROS_NAMESPACE=usb_cam_r1 rosrun image_proc image_proc

ROS_NAMESPACE=usb_cam_r2 rosrun image_proc image_proc


Run LSD SLAM for each robot:
rosrun lsd_slam_core live_slam __name:=lsd_slam_r1 lsd_slam/keyframes:=lsd_slam_r1/keyframes lsd_slam/graph:=lsd_slam_r1/graph lsd_slam/pose:=lsd_slam_r1/pose lsd_slam/liveframes:=lsd_slam_r1/liveframes lsd_slam/debug:=lsd_slam_r1/debug camera_info:=/usb_cam_r1/camera_info image:=/usb_cam_r1/image_rect

rosrun lsd_slam_core live_slam __name:=lsd_slam_r2 lsd_slam/keyframes:=lsd_slam_r2/keyframes lsd_slam/graph:=lsd_slam_r2/graph lsd_slam/pose:=lsd_slam_r2/pose lsd_slam/liveframes:=lsd_slam_r2/liveframes lsd_slam/debug:=lsd_slam_r2/debug camera_info:=/usb_cam_r2/camera_info image:=/usb_cam_r2/image_rect


Run SlaveRobot node on each robot:
cd ~/ROS_HKUST_project/slave_robot1_catkin_ws
source devel/setup.bash
rosrun slave_robot slave_robot __name:=slave_robot1

cd ~/ROS_HKUST_project/slave_robot2_catkin_ws
source devel/setup.bash
rosrun slave_robot slave_robot __name:=slave_robot2


Run main program on central computer:
cd ~/ROS_HKUST_project/CCFabMap_catkin_ws
source devel/setup.bash
rosrun cc_fabmap cc_fabmap_node


Run rviz for visualization:
rosrun rviz rviz
Alternatively if this does not work (core dumped), run it in gdb mode (need to press 'r' and Enter once during startup):
gdb /opt/ros/hydro/lib/rviz/rviz 

