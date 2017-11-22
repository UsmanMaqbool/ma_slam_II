#include "SyncListener.h"


using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;

void lsdSLAMIDToFID(CentralStorage* storage, int lsdSLAMID, int & fID) {
	bool found = false;
	typedef std::map<int, keyFrame> kFMap;
	for (kFMap::iterator it = storage->kFrames.begin(); it != storage->kFrames.end(); ++it) {
		
		if (found == true) {
			continue;   //to break the loop
		}
		if (it->second.lsdSLAMID == lsdSLAMID) {
			fID = it->second.fID;
			found = true;
		}
	}
}
// constructor initializing robot
SyncListener::SyncListener(
	int rID,
    string camInfoTopicName,
    string ptClTopicName,
    string kFTopicName,
    string kFGTopicName,
    ros::NodeHandle nh,
    CentralStorage* storage) : 	m_rID(rID),
								m_nh(nh),
								m_subFilCamInfo(nh, camInfoTopicName, 100),
								m_subFilPtCl(nh, ptClTopicName, 100),
								m_kF(nh, kFTopicName, 100),
								m_kFG(nh, kFGTopicName, 100),
								m_sync(MySyncPolicy(100), m_subFilCamInfo, m_subFilPtCl, m_kF, m_kFG),
								m_localPtCl(new pcl::PointCloud<pcl::PointXYZ>)
							{
								 
								m_sync.registerCallback(boost::bind(&SyncListener::callback_SyncListener, this, _1, _2, _3, _4, storage));
																					// m_subFilCamInfo, m_subFilPtCl, m_kF, m_kFG
								m_originalInput = NULL;
								m_kFrame.img =  cv::Mat(480, 640, CV_8UC3, Scalar(0, 0, 255));

							}

SyncListener::~SyncListener() {}


void SyncListener::callback_SyncListener(
    const CameraInfoConstPtr& camInfoMsg,
    const PointCloud2ConstPtr& ptClMsg,
    const ma_slam::keyframeMsgStamped::ConstPtr& kFMsg,
    const ma_slam::keyframeGraphMsgStamped::ConstPtr& kFGMsg,
    CentralStorage* storage)
{

	// Camera Info - Coming from the usb_cam_r1 etc
	// cam mat stays constant --> no need to process anything
	storage->camModel.fromCameraInfo(camInfoMsg);    //defined in the CentralStorage.h ,l84

	// PtCl Listener Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	//   pcl::fromROSMsg (*input, cloud);

	pcl::fromROSMsg( *ptClMsg, *this->m_localPtCl );
	// Downsample PtCl
	//this->voxelFilterPtCl(storage);

	// kFMsg Listener
	// kFMsg contains the coordinate Transformation from Camera Frame of the corresponding robot to the initial robot frame
	// By taking the inverse, the pose of the Frame is revealed
	this->readkFMsg(kFMsg);



	this->m_kFrame.rID = this->m_rID;


//	cout << "Sync Call back Robot ID: " << m_rID << endl;


	//kFGraphMsg Listener
	this->readkFGMsg(kFGMsg, storage);

	(storage->detector)->detect(this->m_kFrame.img, this->m_kFrame.KPts);

	// this->m_kFrame.camToRobot = tRobotCam = pCamRobot
	// m_localPose = pCamRobot =
	// @input: camera matrix (trafoFloat)
	// @output: Quaternion and traslational and pose scale
 
	HelperFcts::trafoFloat7ToQuatTranslScale(this->m_kFrame.camToRobot, this->m_sim3LocalPoseQuat, this->m_sim3LocalPoseTransl, this->m_sim3LocalPoseScale);
	// get Pose from coordinate transformation (message contains a coordinate transformation!!!)
	//HelperFcts::invQuatTranslScale(this->m_sim3LocalPoseQuat, this->m_sim3LocalPoseQuat, this->m_sim3LocalPoseTransl, this->m_sim3LocalPoseTransl, this->m_sim3LocalPoseScale, this->m_sim3LocalPoseScale);
	
	HelperFcts::eigenQuatTranslPoseToROSPose(this->m_sim3LocalPoseQuat, this->m_sim3LocalPoseTransl, this->m_localPose);


	// JUST FOR CHECKING IF SOPHUS EQUAL OWN IMPLEMENTATION
	/*	memcpy(m_sim3LocalPose.data(), simTrafoMsg->sim3f.data(), 7*sizeof(float));
		cout << endl << endl<< endl;
		cout << "Sophus: " << this->m_sim3LocalPose.translation() << endl;
		cout << this->m_sim3LocalPose.quaternion().w() << " " << this->m_sim3LocalPose.quaternion().x() << " " << this->m_sim3LocalPose.quaternion().y() << " " << this->m_sim3LocalPose.quaternion().z() << endl;
		cout << "scale: " << this->m_sim3LocalPose.scale() << endl;
		cout << this->m_sim3LocalPose.quaternion().w()/this->m_sim3LocalPose.scale() << " " << this->m_sim3LocalPose.quaternion().x()/this->m_sim3LocalPose.scale() << " " << this->m_sim3LocalPose.quaternion().y() << " " << this->m_sim3LocalPose.quaternion().z()/this->m_sim3LocalPose.scale() << endl << endl;
		cout << "Eigen: " << this->m_localPose.pose.position.x << " " << this->m_localPose.pose.position.y << " " << this->m_localPose.pose.position.z << endl;
		cout << this->m_localPose.pose.orientation.w << " " << this->m_localPose.pose.orientation.x << " " << this->m_localPose.pose.orientation.y << " " << this->m_localPose.pose.orientation.z << endl;
		cout << "scale: " << this->m_sim3LocalPoseScale << endl;
		cout << endl << endl<< endl;
	*/
}

void SyncListener::readkFMsg(const ma_slam::keyframeMsgStamped::ConstPtr& kFMsg) {

	for (int i = 0 ; i < 7 ; ++i) {
		this->m_kFrame.camToRobot[i] = kFMsg->camToWorld[i];
	}
	//cout << "i'm in the SyncListener::readkFMsg" << endl;
	this->m_kFrame.width = kFMsg->width;
	this->m_kFrame.height = kFMsg->height;
	this->m_kFrame.fx = kFMsg->fx;
	this->m_kFrame.fy = kFMsg->fy;
	this->m_kFrame.cx = kFMsg->cx;
	this->m_kFrame.cy = kFMsg->cy;
	this->m_kFrame.lsdSLAMID = kFMsg->id;

	cout << "Keyframe message ID" << kFMsg->id << endl;

			// emptying the object
	if (this->m_originalInput != 0) {
		delete[] this->m_originalInput;
	}

	this->m_originalInput = 0;
 	// InputPointDense =>float idepth;, float idepth_var, uchar color[4];

	if (kFMsg->pointcloud.size() != this->m_kFrame.width * this->m_kFrame.height * sizeof(InputPointDense)) {
		if (kFMsg->pointcloud.size() != 0)
		{
			cout << "WARNING" << endl;
			//printf("WARNING: PC with points, but number of points not right! (is %zu, should be %u*%dx%d=%u)\n",
			//kFMsg->pointcloud.size(), sizeof(InputPointDense), this->m_kFrame.width, this->m_kFrame.height, this->m_kFrame.width*this->m_kFrame.height*sizeof(InputPointDense));
		}
	}
	else {
		this->m_originalInput = new InputPointDense[this->m_kFrame.width * this->m_kFrame.height];
		memcpy(	this->m_originalInput, kFMsg->pointcloud.data(), this->m_kFrame.width * this->m_kFrame.height * sizeof(InputPointDense));
	}

	this->m_kFrame.img = cv::Mat(480, 640, CV_8UC3, Scalar(0, 0, 255));
	this->m_kFrame.idepth = cv::Mat(480, 640, CV_32FC1, Scalar(0, 0, 255));
	this->m_kFrame.idepthVar = cv::Mat(480, 640, CV_32FC1, Scalar(0, 0, 255));
	for ( int y = 0 ; y < this->m_kFrame.height - 1 ; y++ ) {
		for ( int x = 0 ; x < this->m_kFrame.width - 1 ; x++) {
			cv::Vec3b bgr;

			bgr[0] = this->m_originalInput[x + y * this->m_kFrame.width].color[0];
			bgr[1] = this->m_originalInput[x + y * this->m_kFrame.width].color[1];
			bgr[2] = this->m_originalInput[x + y * this->m_kFrame.width].color[2];

			this->m_kFrame.img.at< Vec3b >( cv::Point(x, y) ) = bgr;   // 3-channel colour image
			this->m_kFrame.idepth.at< float >(y, x) = this->m_originalInput->idepth;
			this->m_kFrame.idepthVar.at< float >(y, x) = this->m_originalInput->idepth_var;
			
		}
	}
///	cout << "we are reading the kFGraphMsgStamped" << endl;
//	imshow( "m_kFrame.img", this->m_kFrame.img );
//	imshow( "m_kFrame.idepth", this->m_kFrame.idepth );
	//imshow( "m_kFrame.idepthVar", this->m_kFrame.idepthVar );
}

void SyncListener::readkFGMsg(const ma_slam::keyframeGraphMsgStamped::ConstPtr& kFGMsg, CentralStorage * storage) {
	// Get information for graph
	GraphFramePose* graphPoses = (GraphFramePose*)kFGMsg->frameData.data();
	int numGraphPoses = kFGMsg->numFrames;

	cout << "numFrames " << kFGMsg->numFrames << endl;
//	cout << "numConstraints " << kFGMsg->numConstraints << endl;

	for ( int nbrNode = 0 ; nbrNode < numGraphPoses ; nbrNode++ ) {
		int fID = -1000;
		lsdSLAMIDToFID(storage, graphPoses[nbrNode].id, fID);
	//	cout << "graphPoses[nbrNode].id: " << graphPoses[nbrNode].id << " fID: " << fID << endl;

		if (fID == -1000) {
			//	printf("ERROR: graph update contains pose for frame %d, but I dont have a frame %d!\n", graphPoses[nbrNode].id, graphPoses[nbrNode].id);
		}
		else {
			for (int i = 0; i < 7; ++i) {
				nodes[fID].camToOrigin[i] = graphPoses[nbrNode].camToWorld[i];
			}
		}
	}
}

void SyncListener::voxelFilterPtCl(CentralStorage* storage) {

	// Create the filtering object
	pcl::VoxelGrid< pcl::PointXYZ > sor;

	sor.setInputCloud ( (this->m_localPtCl) );
	sor.setLeafSize (0.001f, 0.001f, 0.001f);
	sor.filter ( *(this->m_localPtCl) );

}

void SyncListener::clearData() {

	for ( int i = 0; i < 7; ++i ) {
		this->m_kFrame.camToRobot[i] = 0.0;
		if (i == 3) {
			this->m_kFrame.camToRobot[i] = 1.0;
		}
	}
	this->m_kFrame.cx = 0;
	this->m_kFrame.cy = 0;
	this->m_kFrame.fx = 0;
	this->m_kFrame.fy = 0;
	this->m_kFrame.height = 0;
	this->m_kFrame.width = 0;
	this->m_kFrame.idepth.release();
	this->m_kFrame.idepthVar.release();
	this->m_kFrame.img.release();
	this->m_kFrame.KPts.clear();
	this->m_kFrame.bowDescriptor.release();
	this->m_kFrame.fID = 0;
	this->m_kFrame.rID = 0;
	this->m_kFrame.lsdSLAMID = 0;

	this->m_testKPts.clear();
	this->m_localPtCl->clear();
	this->m_sim3LocalPoseTransl.x() = this->m_sim3LocalPoseTransl.y() = this->m_sim3LocalPoseTransl.z() = 0.0;
	this->m_sim3LocalPoseQuat.setIdentity();
	this->m_sim3LocalPoseScale = 0.0;

	this->m_originalInput = NULL;

	this->nodes.clear();
	this->edges.clear();
}

