#ifndef CENTRALSTORAGE_H
#define CENTRALSTORAGE_H


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <algorithm>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <cstdio>

#include <geometry_msgs/TransformStamped.h>
//#include <tf/transform_datatypes.h>
#include <image_geometry/pinhole_camera_model.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
// include to point cloud to ply
#include <pcl/impl/point_types.hpp>
#include <pcl/io/ply_io.h>

#include "HelperFcts.h"
#include "KalmanFilterScale.h"
#include "PoseGraph.h"
#include "3dcov/cbshot.h"




using namespace std;
using namespace cv;
using namespace opengv;

/**
 * Structure containing all information concerning to one Frame
 */
struct keyFrame {

	// image, bag of words descriptor of image, inverse depth values of image, variance of inverse depth values of image
	Mat img, bowDescriptor, idepth, idepthVar;

	// rID: 		robot ID
	// fID: 		frame ID (testImgCtr)
	// lsdSLAMID: 	frame ID from LSDSLAM - used to for pose graph
	// width: 		width of img
	// height:		height of img
	int rID, fID, lsdSLAMID, width, height;

	// camera parameters corresponding to the image
	image_geometry::PinholeCameraModel camModel; //later
	float fx, fy, cx, cy;

	// extracted key points of the image
	vector<KeyPoint> KPts;

	// cam refers to the camera frame and robot refers to the INITIAL robot frame
	float camToRobot[7];

};


/*! \class CentralStorage
 * \brief Class storing all images of all robots, global maps (pose graph and point cloud)
 */
class CentralStorage {

public:

	int nbrRobots;
	int tnode_no, qnode_no, node_no;
	image_geometry::PinholeCameraModel camModel;


	bool stopDataCollection; // used to control amount of training data
	cv::Mat trainDescriptors_allRobots;	// SIFT descriptors of test Imgs
	vector<vector<KeyPoint> > trainKPts_allRobots; // KPts are in same order as trainImgs
	vector<cv::Mat> trainImgs_allRobots; // KPts are in same order as trainImgs
	int nbrVisWords; // nbr of visual words (cluster centers)

	cv::Mat vocabulary; // vocabulary for FABMAP
	cv::Mat trainBOWDescriptors_allRobots; // BOW descriptors of training images using vocabulary
	cv::Mat clTree; // chow liu tree used to represent joint probab. distr. of visual words for FABMAP



	// Feature detector, extractor, matcher, K-Means trainer, BOW Img Descriptor extractor
	BOWKMeansTrainer* BOWTrainer;
	Ptr<FeatureDetector> detector;
	Ptr<DescriptorExtractor> extractor;
	Ptr<DescriptorMatcher> matcher;
	BOWImgDescriptorExtractor* bide;
	// FABMAP
	Ptr<of2::FabMap> fabmap;

	/// kFrames stores the keyFrame Image, its identity number (fID=ImgCtr), the associated robot (rID), the kPts and the bowDescriptor of the keyFrame
	/// note that the key of the kFrames map is identical to kFrames[key].fID
	std::map<int, keyFrame> kFrames;

	std::map<int, int> rID_ImgCtr_current;
	std::vector<int> robot_1_nextMaches;
	std::vector<int> robot_2_nextMaches;
	
	/// the key for accessing a ptCls corresponds to the the kFrames identity number (ImgCtr), the std::pair contains the associated rID and the actual PtCl
	std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > > ptCls; // map key: ImgCtr, std::pair<int,pcl>: int -> robot ID, pcl -> pcl
	std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > > ptClsInGlobalWorldCOS; // map key: ImgCtr, std::pair<int,pcl>: int -> robot ID, pcl -> pcl
	std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > > GlobalWorldPtCls; // map key: ImgCtr, std::pair<int,pcl>: int -> robot ID, pcl -> pcl
	std::map<int, bool> ptClsInGlobalWorldCOSUpdated; // map key: robot ID rID, bool: true if ptcls coming from robot with robot ID rID are updated (ptcls are already transformed to world COS system by transform found after loop closure)

	/// the key for accessing a local Poses (poses of the keyframe concerning the initial pose of the associated robot)
	/// corresponds to the the kFrames number (ImgCtr), the std::pair contains the associated rID and the actual Transformation
	std::map<int, std::pair <int, geometry_msgs::PoseStamped> > localPoses; // <imgctr, <riD, pose>>
	/// Saves the scales for all robots and all images. IMPORTANT NOTE: only the latest scale is needed for each robot!!!! (current storage->testImgCtr!!!) <-- Maybe not true (see LSD SLAM 3.5 scale for each keyframe)
	std::map<int, std::pair <int, float> > localScales;
	// Kalman Filter to estimate scale difference between maps (robotScales)
	KalmanFilterScale filter;
	// the key corresponds to the robot ID, the second argument contains the scale
	std::map<int, float> robotScales;

	/// the key for accessing a global poses of the robots (pose of the initial robot COS concerning the world COS)
	/// corresponds to the the rID,
	/// the map contains the transformation of the associated robot relative to the world COS
	/// NOTE: The world COS is identical with the COS of the robot with rID 1!!!!!!!!!!
	std::map<int, Eigen::Matrix4f> pRobotsWorld;
	std::map<int, Eigen::Matrix4f> OriginalPoses; // <Nodes renamed , Pose>
	std::map<int, Eigen::Matrix4f> OptimizedPosesinWorld; //<Nodes renamed , Pose> //output of the ceres after optimization
	std::map<int, std::pair <int, Eigen::Matrix4f> > localOriginalPoses; //<Nodes , <robot ID, Pose> //output of the ceres before optimization

	// localOptimizedPoses: <Nodes , <robot ID, Pose>
	std::map<int, std::pair <int, Eigen::Matrix4f> > localOptimizedPoses; //<Nodes , <robot ID, Pose> //output of the ceres after optimization
	
	std::map<int, Eigen::Matrix4f> OptimizedPoses_Edges; //<Nodes original, Edge>
	/// the key for accessing the localPoseGraph corresponding to a robot is the corresponding robot ID rID
	std::map<int, PoseGraph> localPoseGraphs;
	/// all local PoseGraphs added together
	PoseGraph globalPoseGraph;


	cv::Mat testBOWDescriptors_allRobots;
	int testImgCtr;
	vector<of2::IMatch> matches;

	bool boolMatch; // registers if there is a match/ "loop closure"

	bool bootnextfivematches; // if true and it will start counting for each keyframe, (collecting 5 next keyframes)


	std::pair <int, int> matchedKeyFrames; // First keyFrame Number is always associated to queryImg and is equal to the access key of the std::map matchedKeyFramesMap
	/// key is the nbr of the queryImg (ImgCtr) --> access kFrame: storage->kFrames[ matchedKeyFramesMap[nbr].first ], storage->kFrames[ matchedKeyFramesMap[nbr].second ]
	std::map<int, std::pair <int, int> > matchedKeyFramesMap;
	int matchedKFMapDiff; // it will store the value of last two maches differences

	/// key is the nbr of the queryImg (ImgCtr), iGMap.first: queryImgCtr (queryImgCtr), iGMap.second.first:  IG between 2 keyframes, iGMap.second.second: testImgCtr (testImgCtr of matched kFrame)
	/// it contains the pose of the TestImg (2nd) relative to the QueryImg (1st)
	std::map<int, std::pair <Eigen::Matrix4f, int> > iGMap; // QueryImg, <Transformation, Test>
	std::map<int, std::pair <Eigen::Matrix4f, int> > iGMap_croxx; // QueryImg, <Transformation, Test>
	std::map<int, std::pair <Eigen::MatrixXd, int> > iGMap_cov; // QueryImg, <Covariance, Test>
	std::map<int, std::pair <double, int> > match_Probability;
	std::map<int, std::pair <string, int> > match_edge; // <queryimage, <edges, testimage>>
	std::map<int, std::pair <string, int> > match_edge_leftdownright; // <queryimage, <edges, testimage>>
	std::map<int, std::pair <string, int> > match_edge_rightdownleft; // <queryimage, <edges, testimage>>



	//std::map<int,string> SE3_Vertices; //<node#,g2o transformation>
	std::vector<string> SE3_g2o, Matching_info;		//<Id, nodes information and all transformations>
	std::map<int, std::pair <int, geometry_msgs::PoseStamped> > expected_Poses;

	std::map <int, int> rename_Nodes; // <previous, new_assigned>

	Eigen::Matrix4f initial_Guess, Final_Guess, Final_iG, Pose_expected, Pose_diff, P_TestPose, P_QueryPose;
	double iG_match;
	int iG_match_query, iG_match_test;
	int iG_queryImgNbr, iG_testImgNbr, iG_RobotID, iG_Limit, riD_queryImgNbr;



	/*! \fn const char CentralStorage::CentralStorage()
	* \brief Constructor setting initial values to member variables and initializing detector, extractor, matcher and BOWKMeansTrainer
	*/
	CentralStorage(int nbrRobotsInput, int nbrVisWordsInput);
	~CentralStorage();




	/*! \fn const char CentralStorage::generateVocabulary()
	* \brief Member function generating vocabulatory (dictionary/ visual words/ ...) and outputting it in "vocab.yml", also saves all descriptors (SURF) of all robots in "trainDescriptors_allRobots.yml"
	*/
	void generateVocabulary();

	void computeTrainBOWDescriptors();

	/*! \fn const char CentralStorage::generateCLTree()
	* \brief Member function generating chow liu tree and outputting it in "cltree.yml"
	*/
	void generateCLTree();

	void searchForGoodMatch();

	/*! \fn const char CentralStorage::findTrafoInitialGuess()
	* \brief Member function calculating an initial guess for the pose of one camera relative to the other camera of two matched images based on matched keypoints
	* The result of running, namely pTestQuery (p2nd1st), is stored in storage->iGMap[queryImgCtr]
	*/
	void findTrafoInitialGuess();

	void findnextmatches();
	void calculateMatching(int queryImgNbr, int testImgNbr, string & edges3d);
	void Apply_Optimization();
	/*! \fn const char CentralStorage::findTrafo(Eigen::Matrix4f iG, Eigen::Matrix4f & finalTrafo)
	* \brief Member function calculating an the pose of one camera relative to the other camera of two matched images based icp with the initial Guess calculated in findTrafoInitialGuess
	* The result of running, namely pTestQuery (p2nd1st), is stored in ????
	*/
	void findTrafo();

	void estimateScale();

	void updatePtCls();

	void updatePosesOfRobots();
	void PointCloud_Fusion();
	void clearData();

};





#endif
