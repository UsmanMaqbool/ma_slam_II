# ma_slam with 3dconvarience
## Training

1. For CC_FABMAP training, firstly open the cc_fabmap_node.cpp and make `bool doTraining = true` in line 318, cc_fabmap_node.cpp and compile the package using the `catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo`

2. Open new terminal and Run `roscore`

3. Open new terminal and Run `roslaunch ueye_cam bayer_rggb_image_proc.launch`
**topics must be same**
 "/narrow_stereo/image_rect" and "/narrow_stereo/image_raw", you can edit the topics in line 350 of cc_fabmap_node.cpp according to your cam topics.
<!--  # you can check the camera topic using $ rostopic list -->
4. Open new terminal and Run `$ rosrun image_view image_view image:=/narrow_stereo/image_raw`   <!-- to view the live image -->

5. Open new terminal and Run `ROS_NAMESPACE=narrow_stereo rosrun image_proc image_proc`   <!-- to get /narrow_stereo/image_rect -->
6. Open new terminal and go to the directory where the vocabulray files to be saved. for example `cd /home/leo/awc_pc_fusion/src/cc_fabmap/fabmap_vocabulary`
7. Open new terminal and Run `rosrun cc_fabmap cc_fabmap_node`


## Record Bag Files
1. Open new terminal and go to the directory where the vocabluray files to be saved. for example;
   `cd /home/leo/awc_pc_fusion/src/bag_files`
2. `roslaunch ueye_cam bayer_rggb_image_proc.launch` 
<! --
to view;
rosrun image_view image_view image:=/narrow_stereo/image_raw

# it will show nothing for /narrow_stereo/image_rect, to run and check firslty we have to rectify it

ROS_NAMESPACE=narrow_stereo rosrun image_proc image_proc
rosrun image_view image_view image:=/narrow_stereo/image_rect
 -->
ROS_NAMESPACE=narrow_stereo rosrun image_proc image_proc
rosrun lsd_slam_core live_slam camera_info:=/narrow_stereo/camera_info image:=/narrow_stereo/image_rect
5. For `robot1` record
    `rosbag record /narrow_stereo/camera_info /narrow_stereo/image_raw -o robot_pc1.bag`
6. For `robot2` record
    `rosbag record /narrow_stereo/camera_info /narrow_stereo/image_raw -o robot_pc1.bag`
7. Edit the nodes values according to the usages in cc_fab_map and LSD_SLAM
#fisheyecam
rosrun rosbag topic_renamer.py /narrow_stereo/image_rect training_bag1.bag /usb_cam_training/image_rect train_video1.bag
rosrun rosbag topic_renamer.py /narrow_stereo/image_raw training_bag2.bag /usb_cam_training_robot2/image_raw train_video2.bag

#ueyecam
`Robot1`
rosrun rosbag topic_renamer.py /narrow_stereo/camera_info ueye9.bag /usb_cam_r1/camera_info video11_temp.bag
rosrun rosbag topic_renamer.py /narrow_stereo/image_rect video11_temp.bag /usb_cam_r1/image_rect ueye9.bag
`Robot2`
rosrun rosbag topic_renamer.py /narrow_stereo/camera_info ueye10.bag /usb_cam_r2/camera_info video12_temp.bag
rosrun rosbag topic_renamer.py /narrow_stereo/image_rect video12_temp.bag /usb_cam_r2/image_rect ueye10.bag



# to make direct image rect file
rosrun rosbag topic_renamer.py /narrow_stereo/camera_info ueye99.bag /usb_cam_r1/camera_info video12_temp.bag 
rosrun rosbag topic_renamer.py /narrow_stereo/image_rect video12_temp.bag /usb_cam_r1/image_rect ueye9.bag

#to confirm, the topic is changed now....
rosbag info train_video1.bag
