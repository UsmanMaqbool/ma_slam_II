#ifndef SYNCLISTENER_H
#define SYNCLISTENER_H

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>

#include "ma_slam/keyframeMsgStamped.h"
#include "ma_slam/keyframeGraphMsgStamped.h"
#include "ma_slam/keyframeMsg.h"
#include "ma_slam/keyframeGraphMsg.h"

#include <cv_bridge/cv_bridge.h>

// PtCl filtering
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// JUST FOR CHECKING IF SOPHUS EQUAL OWN IMPLEMENTATION
// #include "sophus/sim3.hpp"

#include "CentralStorage.h"


using namespace std;
using namespace std_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;

// Struct to help reading ROS kFMessage from LSD SLAM
struct InputPointDense {
	float idepth;
	float idepth_var;
	uchar color[4];
};

// Struct to help reading ROS kFGMessages from LSD SLAM
struct GraphFramePose {
	int id;
	float camToWorld[7];
};
// Struct to help reading ROS kFGMessages from LSD SLAM
struct GraphConstraint
{
	int from;
	int to;
	float err;
};

/*! \class SyncListener
 * \brief Class listening to incoming Msgs and synchronizes them, each instance of the class listens to one robot
 */
class SyncListener
{
public:

	int m_rID;

	ros::NodeHandle m_nh;

	message_filters::Subscriber<CameraInfo> m_subFilCamInfo;
	message_filters::Subscriber<PointCloud2> m_subFilPtCl;
	message_filters::Subscriber<ma_slam::keyframeMsgStamped> m_kF;
	message_filters::Subscriber<ma_slam::keyframeGraphMsgStamped> m_kFG;

	typedef sync_policies::ApproximateTime<	CameraInfo, PointCloud2, ma_slam::keyframeMsgStamped, ma_slam::keyframeGraphMsgStamped> MySyncPolicy;
	Synchronizer<MySyncPolicy> m_sync;



	// PtCl
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_localPtCl;

	// key Frame
	keyFrame m_kFrame; ///< stores necessary information about one keyframe
	cv_bridge::CvImagePtr m_testCvPtr; ///< save ros sensor img to opencv img
	vector<KeyPoint> m_testKPts; ///< vec<KPts>: all kpts from 1 testImg of 1 robot

	geometry_msgs::PoseStamped m_localPose;

	Eigen::Translation<float,3> m_sim3LocalPoseTransl;
	Eigen::Quaternion<float> m_sim3LocalPoseQuat;
	float m_sim3LocalPoseScale;

	InputPointDense * m_originalInput;

	// key Frame graph
	std::map<int,Node> nodes;
	std::map<int,Edge> edges;


//	JUST FOR CHECKING IF SOPHUS EQUAL OWN IMPLEMENTATION
//	Sophus::Sim3f m_sim3LocalPose;









	/*! \fn const char SyncListener::SyncListener()
	* \brief Constructor initializing member variables
	*/
//	SyncListener(ros::NodeHandle _nh);
	SyncListener(int rID,
			string camInfoTopicName,
			string ptClTopicName,
			string kFTopicName,
			string kFGTopicName,
			ros::NodeHandle nh, CentralStorage* storage);
	~SyncListener(); ///< destructor

	/*! \fn const char SyncListener::callback_SyncListener(const CameraInfoConstPtr& camInfoMsg,const PointCloud2ConstPtr& ptClMsg,const ma_slam::keyframeMsgStamped::ConstPtr& kFMsg,const ma_slam::keyframeGraphMsgStamped::ConstPtr& kFGMsg,CentralStorage* storage)
	* \brief Member function listening to incoming Msgs
	*/
	void callback_SyncListener(const CameraInfoConstPtr& camInfoMsg,
			const PointCloud2ConstPtr& ptClMsg,
			const ma_slam::keyframeMsgStamped::ConstPtr& kFMsg,
			const ma_slam::keyframeGraphMsgStamped::ConstPtr& kFGMsg,
			CentralStorage* storage);

	void readkFMsg(const ma_slam::keyframeMsgStamped::ConstPtr& kFMsg);

	void readkFGMsg(const ma_slam::keyframeGraphMsgStamped::ConstPtr& kFGMsg, CentralStorage * storage);

	void voxelFilterPtCl(CentralStorage* storage);

	void clearData();
};


#endif

