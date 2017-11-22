/*
 * This is the main program! Settings and parameters can be adjusted here. Instances of ROS nodes spinning, publishing and subscribing to messages are created here.
 * Furthermore, helper functions dependent on the CentralStorage class are implemented here
 *
 * Overview over other files:
 * CentralStorage.h/.cpp:			Contains the CentralStorage class. Here all important data received from other nodes are stored in variables.
 * KeyFrameListenerTraining.h/.cpp:	When creating an instance of this class, it creates a ROS node listening to images.
 * 									Its result is a vocabulary (BOW/visual words), chow-liu tree and the BOW-descriptors of the training data
 * 									This is used for the FABMAP algorithm
 * SyncListener.h/.cpp:				When creating an instance of this class, it creates a ROS node listening to .... and makes sure that the message are synchronized (approximate time)
 * HelperFcts.h/.cpp				Static helper functions depending only on standard and thirdparty libraries (not on own classes)
 */


#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>


#include "KeyFrameListenerTraining.h"
#include "SyncListener.h"




using namespace std;
using namespace cv;
using namespace opengv;


int skipmatch = 0;

void helperDispOutput(CentralStorage* storage, bool saveMatchMatrix = false, string matchMatrixNameImg = "", string matchMatrixNameFile = "") {
	int nbrImgs = storage->testBOWDescriptors_allRobots.size().height; // query img + test imgs
	int queryCtr = 0;

	Mat match_small_f = Mat::zeros(nbrImgs, nbrImgs, CV_32FC1);
	Mat matchMatrixRGB = Mat::zeros(nbrImgs, nbrImgs, CV_32FC3);

	vector<of2::IMatch>::const_iterator l;
	for (l = storage->matches.begin(); l != storage->matches.end(); l++) {
// USEFUL FOR DEBUG:		cout << "queryIdx: " << l->queryIdx << "   imgIdx: " << l->imgIdx << "   match: " << l->match << endl;
		if (l->imgIdx < 0) {
			match_small_f.at<float>(queryCtr, queryCtr) = (l->match * 1.0);
			if (storage->kFrames.at(queryCtr).rID == 1) {
				matchMatrixRGB.at<Vec3f>(queryCtr, queryCtr).val[0] = l->match * 255.0; // .at(col,row)
			}
			else if (storage->kFrames.at(queryCtr).rID == 2) {
				matchMatrixRGB.at<Vec3f>(queryCtr, queryCtr).val[1] = l->match * 255.0;
			}
			queryCtr = queryCtr + 1;

		} else {
			match_small_f.at<float>(queryCtr - 1, l->imgIdx) = (l->match * 1.0);
			if (storage->kFrames.at(l->imgIdx).rID == 1) {
				matchMatrixRGB.at<Vec3f>(queryCtr - 1, l->imgIdx).val[0] = l->match * 255.0;
			}
			else if (storage->kFrames.at(l->imgIdx).rID == 2) {
				matchMatrixRGB.at<Vec3f>(queryCtr - 1, l->imgIdx).val[1] = l->match * 255.0;
			}
		}
	}

	//cout << match_small_f << endl;
	//Mat match_large_f(10*nbrImgs, 10*nbrImgs, CV_32FC1);
	Mat matchMatrixRGB_large = Mat::zeros(10 * nbrImgs, 10 * nbrImgs, CV_32FC3);

	//resize(match_small_f, match_large_f, Size(500, 500), 0, 0, CV_INTER_NN);
	resize(matchMatrixRGB, matchMatrixRGB_large, Size(500, 500), 0, 0, CV_INTER_NN);

	if (saveMatchMatrix) {
		HelperFcts::saveImage(matchMatrixRGB_large, matchMatrixNameImg);
		HelperFcts::saveMatrix(matchMatrixRGB, matchMatrixNameFile);
	}

	/*
	 * 		I0 I1 I2 I3 ... I23 Q
	 * I0
	 * I1
	 * I2
	 * ...
	 *
	 * I23
	 * Q
	 */
	imshow("Confusion match probab Matrix", matchMatrixRGB_large / 255.0);

	waitKey(5);
}

void helperGetTestData(CentralStorage* storage) {

	//std::vector<keyFrame>::iterator itKF;
	std::map<int, keyFrame>::iterator itKF;
	for (itKF = storage->kFrames.begin(); itKF != storage->kFrames.end(); ++itKF) {
		storage->testBOWDescriptors_allRobots.push_back((*itKF).second.bowDescriptor);
	}

}

void helperPublishAssembledGlobalPtCl(CentralStorage * storage, ros::Publisher & pub, int skipFirstN) {

	//vector<int> rIDToUpdate;
	//typedef std::map<int,bool> ptClUpdatedMap;
	//for (ptClUpdatedMap::iterator it = storage->ptClsInGlobalWorldCOSUpdated.begin(); it!=storage->ptClsInGlobalWorldCOSUpdated.end(); ++it) {
	//	if(it->second == true) {
	//		rIDToUpdate.push_back(it->first);
	//	}
	//}

	//if(!rIDToUpdate.empty()) {
	//	cout << "Not all PtCls up-to-date - no visualization" << endl;
	//}
	//else {
	cout << "	Publishing assembled global PtCl" << endl;

	pcl::PointCloud<pcl::PointXYZ> assembledGlobalPtCl;

	pcl::PointCloud<pcl::PointXYZRGB> assembledGlobalPtClRGB;
	pcl::PointCloud<pcl::PointXYZRGB> PtCl1RGB;
	pcl::PointCloud<pcl::PointXYZRGB> PtCl2RGB;


	std::map<int, int> skipCtr;
	for (int i = 1; i <= storage->nbrRobots; ++i) {
		skipCtr[i] = 0;
	}
	typedef std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > > ptClMap;
	// <robot ID, <int, point cloud>>
	for (ptClMap::iterator it = storage->ptClsInGlobalWorldCOS.begin(); it != storage->ptClsInGlobalWorldCOS.end(); ++it) {
		bool lessThanN = false;
		for (int i = 1; i <= storage->nbrRobots; ++i) {
			if (skipCtr[i] < skipFirstN) {
				lessThanN = true;
			}
		}
		if (lessThanN == false) {
			assembledGlobalPtCl += it->second.second;

			cout << "Adding colors to Grobal Point Cloud" << endl;
			//	uint8_t r = 255, g = 0, b = 0;    // Example: Red color
			if (it->second.first == 1) {

				pcl::copyPointCloud(it->second.second, PtCl1RGB);
				for (size_t i = 0; i < PtCl1RGB.points.size (); ++i) {
					PtCl1RGB.points[i].r = 0;
					PtCl1RGB.points[i].g = 255;
					PtCl1RGB.points[i].b = 0;
				}
				assembledGlobalPtClRGB += PtCl1RGB;
			}
			if (it->second.first == 2) {
				pcl::copyPointCloud(it->second.second, PtCl2RGB);
				for (size_t i = 0; i < PtCl2RGB.points.size (); ++i) {
					PtCl2RGB.points[i].r = 255;
					PtCl2RGB.points[i].g = 0;
					PtCl2RGB.points[i].b = 0;
				}
				assembledGlobalPtClRGB += PtCl2RGB;
			}



		}
		skipCtr[it->second.first] = skipCtr[it->second.first] + 1;
	}
	cout << "globptcl: " << assembledGlobalPtCl.width << endl;
	assembledGlobalPtCl.header.frame_id = "world";
	assembledGlobalPtCl.header.stamp = 0;
	pub.publish(assembledGlobalPtCl);
	pcl::io::savePLYFileASCII ("matchingImgs/assembledGlobalPtCl.ply", assembledGlobalPtCl);
	pcl::io::savePLYFileASCII ("matchingImgs/assembledGlobalPtClRGB.ply", assembledGlobalPtClRGB);


	//}
}

void helperPublishAssembledLocalPtCl(CentralStorage * storage, int rID, ros::Publisher & pub, int skipFirstN) {

	//if(storage->ptClsInGlobalWorldCOSUpdated[rID] == false) {
	//	cout << "PtCl of robot " << rID << " up-to-date - no visualization" << endl;
	//}
	//else {
	cout << "	Publishing assembled local PtCl of Robot with robot ID: " << rID << endl;
	cout << "	size of robot: " << rID << "ptClsInGlobalWorldCOS " << storage->ptClsInGlobalWorldCOS.size() << endl;

	pcl::PointCloud<pcl::PointXYZ> assembledLocalPtCl;// (new pcl::PointCloud<pcl::PointXYZ>());

	int skipCtr = 0;
	typedef std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > > ptClMap;
	for (ptClMap::iterator it = storage->ptClsInGlobalWorldCOS.begin(); it != storage->ptClsInGlobalWorldCOS.end(); ++it) {
		if ( (it->second.first == rID) ) {
			if (skipCtr >= skipFirstN) {
				assembledLocalPtCl += it->second.second;
			}
		}
		skipCtr = skipCtr + 1;
	}
	assembledLocalPtCl.header.frame_id = "world";
	assembledLocalPtCl.header.stamp = 0;
	pub.publish(assembledLocalPtCl);
	if (rID = 1) {
		pcl::io::savePLYFileASCII ("matchingImgs/assembledLocalPtClofRobot1.ply", assembledLocalPtCl);
	}
	if (rID = 2) {
		pcl::io::savePLYFileASCII ("matchingImgs/assembledLocalPtClofRobot2.ply", assembledLocalPtCl);
	}
}

void helperLocalPtCl_publisher(CentralStorage * storage, int rID, ros::Publisher & pub, int skipFirstN) {

	//if(storage->ptClsInGlobalWorldCOSUpdated[rID] == false) {
	//	cout << "PtCl of robot " << rID << " up-to-date - no visualization" << endl;
	//}
	//else {
	cout << "	Publishing assembled local PtCl of Robot with robot ID: " << rID << endl;
	cout << "	size of robot: " << rID << "ptClsInGlobalWorldCOS " << storage->ptClsInGlobalWorldCOS.size() << endl;

	pcl::PointCloud<pcl::PointXYZ> assembledLocalPtCl;// (new pcl::PointCloud<pcl::PointXYZ>());

	int skipCtr = 0;
	typedef std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > > ptClMap;
	for (ptClMap::iterator it = storage->ptCls.begin(); it != storage->ptCls.end(); ++it) {
		if ( (it->second.first == rID) ) {
			if (skipCtr >= skipFirstN) {
				assembledLocalPtCl += it->second.second;
			}
		}
		skipCtr = skipCtr + 1;
	}
	assembledLocalPtCl.header.frame_id = "world";
	assembledLocalPtCl.header.stamp = 0;
	pub.publish(assembledLocalPtCl);
	//}
}

void helperPublishPtCl(pcl::PointCloud<pcl::PointXYZ> ptCl, ros::Publisher & pub) {
	pcl::PointCloud<pcl::PointXYZ> ptClToPublish;
	ptClToPublish = ptCl;

	ptClToPublish.header.frame_id = "world";
	ptClToPublish.header.stamp = 0;

	pub.publish(ptClToPublish);
}

void helperReadTrainData(CentralStorage* storage) {

	/*
		// load training Images
	//	fs.open(string("trainImgs_allRobots.yml"), FileStorage::READ);
	//	fs["trainImgs_allRobots"] >> storage->trainImgs_allRobots;
	//	if (storage->trainImgs_allRobots.empty()) {
	//		cerr << "trainImgs_allRobots not found" << endl;
	//		return -1;
	//	}
	//	fs.release();

		// load training KeyPoints
	//	fs.open(string("trainKPts_allRobots.yml"), FileStorage::READ);
	//	fs["trainKPts_allRobots"] >> storage->trainKPts_allRobots;
	//	if (storage->trainKPts_allRobots.empty()) {
	//		cerr << "trainKPts_allRobots not found" << endl;
	//		return -1;
	//	}
	//	fs.release();

		// load SURF descriptors of training data
	//	fs.open(string("trainDescriptors_allRobots.yml"), FileStorage::READ);
	//	fs["trainDescriptors_allRobots"] >> storage->trainDescriptors_allRobots;
	//	if (storage->trainDescriptors_allRobots.empty()) {
	//		cerr << "Training Data SURF Descriptors not found" << endl;
	//		return -1;
	//	}
	//	fs.release();
	*/

	FileStorage fs;

	// load vocabulary
	fs.open(string("vocabulary.yml"), FileStorage::READ);
	fs["vocabulary"] >> storage->vocabulary;
	if (storage->vocabulary.empty()) {
		cerr << "Vocabulary not found" << endl;
//		return -1;
	}
	fs.release();

	// load trainBOWDescriptors_allRobots
	fs.open(string("trainBOWDescriptors_allRobots.yml"), FileStorage::READ);
	fs["trainBOWDescriptors_allRobots"] >> storage->trainBOWDescriptors_allRobots;
	if (storage->trainBOWDescriptors_allRobots.empty()) {
		cerr << "trainBOWDescriptors_allRobots not found" << endl;
//		return -1;
	}
	fs.release();

	// load clTree
	fs.open(string("clTree.yml"), FileStorage::READ);
	fs["clTree"] >> storage->clTree;
	if (storage->clTree.empty()) {
		cerr << "clTree not found" << endl;
//		return -1;
	}
	fs.release();
}

void helperSaveTestImgs(CentralStorage* storage, SyncListener & listener_r1, SyncListener & listener_r2) {
	// NO IMPLEMENTATION YET
}

void helperSaveTrainData(CentralStorage* storage) {

	FileStorage fs1("vocabulary.yml", FileStorage::WRITE);
	fs1 << "vocabulary" << storage->vocabulary;
	fs1.release();

	FileStorage fs2("trainBOWDescriptors_allRobots.yml", FileStorage::WRITE);
	fs2 << "trainBOWDescriptors_allRobots" << storage->trainBOWDescriptors_allRobots;
	fs2.release();

	FileStorage fs3("clTree.yml", FileStorage::WRITE);
	fs3 << "clTree" << storage->clTree;
	fs3.release();

}

void helperStoreTrainData(CentralStorage* storage, KeyFrameListenerTraining & listenerR1, KeyFrameListenerTraining & listenerR2) {

	storage->trainDescriptors_allRobots.push_back(listenerR1.m_trainImgDescr);
	storage->trainDescriptors_allRobots.push_back(listenerR2.m_trainImgDescr);

	std::vector<cv::Mat>::iterator itImg1;
	for (itImg1 = listenerR1.m_trainImgs.begin(); itImg1 != listenerR1.m_trainImgs.end(); ++itImg1) {
		storage->trainImgs_allRobots.push_back(*itImg1);
	}
	std::vector<cv::Mat>::iterator itImg2;
	for (itImg2 = listenerR2.m_trainImgs.begin(); itImg2 != listenerR2.m_trainImgs.end(); ++itImg2) {
		storage->trainImgs_allRobots.push_back(*itImg2);
	}


	std::vector<vector<KeyPoint> >::iterator itKPts1;
	for (itKPts1 = listenerR1.m_trainKPts.begin(); itKPts1 != listenerR1.m_trainKPts.end(); ++itKPts1) {
		storage->trainKPts_allRobots.push_back(*itKPts1);
	}
	std::vector<vector<KeyPoint> >::iterator itKPts2;
	for (itKPts2 = listenerR2.m_trainKPts.begin(); itKPts2 != listenerR2.m_trainKPts.end(); ++itKPts2) {
		storage->trainKPts_allRobots.push_back(*itKPts2);
	}
}














int main(int argc, char **argv) {
	cout << "Started..." << endl;
// commented by leo
	system("exec rm -r /home/leo/catkin_ws/src/vocabulary_parking/testImgs/*");
	system("exec rm -r /home/leo/catkin_ws/src/vocabulary_parking/matchingImgs/*");

	initModule_nonfree();
//	cout << "line 313 - cc_fabmap_node.cpp" << endl;
	/// ---- INITIALIZE ---------------------------------------------------------
	bool doTraining = false; // enable training (vocabulary, chow-liu tree, create file with training data BOW Img descriptors)
	// ----- above line - Leo_edited = doTraining = false
	bool doTesting = true; // enable testing (gathering ptcls and check for loop closures between different robots and loop closures concerning one robot
	bool boolSaveTestImgs = true; // enable to save the received keyframes
	bool boolNewKF = false; // will be set to true whenever a new keyframe with enough information (enough keypoints) was sent to central computer
	bool boolEndAfterFoundMatch = false; // enable to stop looking for further loop closures after the first match - just for verification of the code
	bool boolVisualizePtCl = true; // enable to visualize the local ptcls of the different robots in RVIZ
	bool boolVisualizedOnce = false;
	int nbrRobots = 2; // Number of robots in the system - aim to be set during runtime
	int nbrVisWords = 700; //bag of words for K- mean trainer
	int nbrTrainImgs = 300;


	// for the syncListner loop
	SyncListener *syncListenerRobot[nbrRobots];
	string Robot_Sync, Robot_Caminfo, Robot_slave, Robot_PointCloud2Filtered, Robot_kFMsgStamped, Robot_kFGraphMsgStamped;


	ros::init(argc, argv, "ma_slam");
	ros::NodeHandle nh;

	CentralStorage* storage = new CentralStorage(nbrRobots, nbrVisWords);


	/**
	 *
	 * Start the reading vocabulary from the folder
	 *
	 */

	storage->stopDataCollection = true;

	helperReadTrainData(storage);

	cout << "Set Vocabulary for BOW Descriptor extractor" << endl;
	(storage->bide)->setVocabulary(storage->vocabulary);

	cout << "Initialize FabMap instance" << endl << endl;
	storage->fabmap = new of2::FabMap2(storage->clTree, 0.4, 0, of2::FabMap::SAMPLED | of2::FabMap::CHOW_LIU);
	storage->fabmap->addTraining(storage->trainBOWDescriptors_allRobots);

	cout << "Vocabulary Loading Finished --------------------------------" << endl << endl;

	/// ----------------------------------------
	/// ----------------------------------------
	///
	/// TESTING --------------------------------
	///
	/// ----------------------------------------
	/// ----------------------------------------
	if (doTesting) {

		cout << "Initialize synchronized ROS topic listener" << endl;
		SyncListener syncListenerR1(1, "/usb_cam_r1/camera_info",
		                            "/slave_robot1/pointcloud2Filtered",
		                            "/slave_robot1/kFMsgStamped",
		                            "/slave_robot1/kFGraphMsgStamped",
		                            nh, storage);
		SyncListener syncListenerR2(2, "/usb_cam_r2/camera_info",
		                            "/slave_robot2/pointcloud2Filtered",
		                            "/slave_robot2/kFMsgStamped",
		                            "/slave_robot2/kFGraphMsgStamped",
		                            nh, storage);





		// ONLY FOR CHECKING IN RVIZ (RUN RVIZ IN GDB MODE IF rosrun rviz rviz does not work: gdb /opt/ros/hydro/lib/rviz/rviz)

		//For publising the pointclouds

		ros::Publisher pub1 = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("ptClCheck1", 1);
		ros::Publisher pub2 = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("ptClCheck2", 1);
		ros::Publisher pub3 = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("ptClCheck3", 1);
		ros::Publisher pub4 = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("ptClCheck4", 1);
		ros::Publisher pub5 = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("ptClCheck5", 1);

		static tf::TransformBroadcaster br1, br2, br3, br4;

		tf::Transform trafoROSMsgR1, trafoROSMsgR2,
		trafoROSMsgR1C1, trafoROSMsgC1R1,
		trafoROSMsgR2C2, trafoROSMsgC2R2, trafoROSMsgR2FromSyncListener,
		trafoROSMsgC2C1, trafoROSMsgC1C2;

		HelperFcts::eigenMatrix4fToROSTrafoMsg(Eigen::Matrix4f::Identity(4, 4), trafoROSMsgR1);
		HelperFcts::eigenMatrix4fToROSTrafoMsg(Eigen::Matrix4f::Identity(4, 4), trafoROSMsgR2);
		HelperFcts::eigenMatrix4fToROSTrafoMsg(Eigen::Matrix4f::Identity(4, 4), trafoROSMsgR1C1);
		HelperFcts::eigenMatrix4fToROSTrafoMsg(Eigen::Matrix4f::Identity(4, 4), trafoROSMsgC1R1);
		HelperFcts::eigenMatrix4fToROSTrafoMsg(Eigen::Matrix4f::Identity(4, 4), trafoROSMsgR2C2);
		HelperFcts::eigenMatrix4fToROSTrafoMsg(Eigen::Matrix4f::Identity(4, 4), trafoROSMsgC2R2);
		HelperFcts::eigenMatrix4fToROSTrafoMsg(Eigen::Matrix4f::Identity(4, 4), trafoROSMsgC2C1);
		HelperFcts::eigenMatrix4fToROSTrafoMsg(Eigen::Matrix4f::Identity(4, 4), trafoROSMsgC1C2);
		// END ONLY FOR CHECKING IN RVIZ --------------------------------------

		// string variables to save imgs, files
		ostringstream convImgName, convMatchMatrixNameImg, convMatchMatrixNameFile;
		string imgName, matchMatrixNameImg, matchMatrixNameFile;

		cout << endl << "TESTING --------------------------------" << endl << endl;
		ros::Rate rTest(15); // adequate spinning frequency???

		while (ros::ok() && storage->testImgCtr < 2000)
		{

			ros::spinOnce();

			// Robot 1 received a nonempty keyframe with enough keypts
			if (!syncListenerR1.m_kFrame.img.empty() && syncListenerR1.m_kFrame.KPts.size() >= 20 ) {

				cout << "---------------------------------------------" << endl << "ROBOT 1 has received a keyframe" << endl;
				//cout << "current image testImgCtr = " << storage->testImgCtr << endl;

				if(storage->bootnextfivematches){

					storage->robot_1_nextMaches.push_back(storage->testImgCtr);
					storage->rID_ImgCtr_current[1] = storage->rID_ImgCtr_current[1] + 1; 
				}
				cout << "Robot 1 is now have keyframes # " << storage->rID_ImgCtr_current[1] << endl;
				if (boolSaveTestImgs) {
					// Defining strings to save imgs, files, ...

					// Clearing all the string variables which actually save the keyframe images
					convImgName.clear(); convImgName.str("");							// for image name
					convMatchMatrixNameImg.clear(); convMatchMatrixNameImg.str("");		// for confusion matrix
					convMatchMatrixNameFile.clear(); convMatchMatrixNameFile.str("");	// for generating the yml file

					// Creating the image Name
					convImgName << "testImgs/robot" << syncListenerR1.m_rID << "TestImg" << storage->testImgCtr << ".jpg";
					convMatchMatrixNameImg << "testImgs/robot" << syncListenerR1.m_rID << "MatchMatrix" << storage->testImgCtr << ".jpg";
					convMatchMatrixNameFile << "testImgs/robot" << syncListenerR1.m_rID << "MatchMatrix" << storage->testImgCtr << ".yml";


					// concatination the names
					imgName = convImgName.str();
					matchMatrixNameImg = convMatchMatrixNameImg.str();
					matchMatrixNameFile = convMatchMatrixNameFile.str();
					HelperFcts::saveImage(syncListenerR1.m_kFrame.img, imgName);
				}


				// Compute BOW descriptor of current query image
				syncListenerR1.m_kFrame.fID = storage->testImgCtr;
				storage->bide->compute(syncListenerR1.m_kFrame.img, syncListenerR1.m_kFrame.KPts, syncListenerR1.m_kFrame.bowDescriptor);

				// Add Keyframe to storage
				storage->kFrames[storage->testImgCtr] = syncListenerR1.m_kFrame;

				// Add ptcl to storage (robot ID, Point Cloud)
				// `std::map<int,std::pair <int,pcl::PointCloud<pcl::PointXYZ> > >`

				storage->ptCls[storage->testImgCtr] = std::make_pair( syncListenerR1.m_rID, *(syncListenerR1.m_localPtCl) );
				syncListenerR1.m_localPtCl->clear();
		     	//	cout << "-------------" << endl << "ROBOT 1+2 has received a total ptCls= " << storage->ptCls.size() << endl;

				// Add keyframe pose in initial robot COS to storage
				storage->localPoses[storage->testImgCtr] = std::make_pair(syncListenerR1.m_rID, syncListenerR1.m_localPose);
				storage->localScales[storage->testImgCtr] = std::make_pair(syncListenerR1.m_rID, syncListenerR1.m_sim3LocalPoseScale);

				// RUN FABMAP ROBOT 1
				cout << "	Running FABMAP" << endl;
				storage->testBOWDescriptors_allRobots.push_back(syncListenerR1.m_kFrame.bowDescriptor);
				storage->fabmap->compare(syncListenerR1.m_kFrame.bowDescriptor, storage->matches, true);


				if (!boolEndAfterFoundMatch) {

					// SEARCH FOR BEST MATCH
					storage->searchForGoodMatch();
					// Compute transformation between matched Images and subsequently between PtCls
					if (storage->boolMatch == true) // && storage->matchedKFMapDiff > 0 && storage->matchedKFMapDiff < 15)
					{
						//		cout << "	Robot1 found a loop-closure" << endl;

						skipmatch++;
						storage->findTrafoInitialGuess(); // `uthis->initial_Guess` // if this otherwise push next transformation
						
						
						
						storage->boolMatch = false;
						storage->bootnextfivematches = true;
							
					}
					if (storage->rID_ImgCtr_current[1] > 4 && storage->rID_ImgCtr_current[2] > 4) {
					storage->findnextmatches();
					}
					if (storage->iGMap.size() > 5) // skipmatch == 1) // && storage->matchedKFMapDiff < 7)
					{
						// find ICP transform
						//  storage->findTrafo();

						storage->Apply_Optimization();

						
						HelperFcts::eigenMatrix4fToROSTrafoMsg( storage->iGMap[storage->testImgCtr].first, trafoROSMsgC2C1 ); // not used


						boolEndAfterFoundMatch = true;
						storage->bootnextfivematches = false;
					}
				}
				cout << "size of iGMap is = " << storage->iGMap.size() << endl;
				storage->updatePosesOfRobots(); 		// update this: this->pRobotsWorld
				storage->updatePtCls();					// use the pRobotsWorld to transform the point cloud

				Eigen::Matrix4f pCam1Robot1(Eigen::Matrix4f::Identity(4, 4)), pRobot1Cam1(Eigen::Matrix4f::Identity(4, 4));
				HelperFcts::poseStampedROSToMatrix4f(storage->localPoses[storage->testImgCtr].second, pCam1Robot1);
				HelperFcts::eigenMatrix4fToROSTrafoMsg(pCam1Robot1, trafoROSMsgC1R1);
				br1.sendTransform(tf::StampedTransform( trafoROSMsgC1R1, ros::Time::now(), "world", "cam1") );


				boolNewKF = true;

				storage->testImgCtr = storage->testImgCtr + 1;
				//	helperLocalPtCl_publisher(storage, 1, pub4, 2);
				cout << "---------------------------------------------" << endl << endl << endl;
			} else if (!syncListenerR1.m_kFrame.img.empty() && syncListenerR1.m_kFrame.KPts.size() < 20 ) {
				if(storage->bootnextfivematches){
					storage->robot_1_nextMaches.push_back(storage->testImgCtr);
					storage->rID_ImgCtr_current[1] = storage->rID_ImgCtr_current[1] + 1; 
				}
				cout << "Robot 1 is now have keyframes # " << storage->rID_ImgCtr_current[1] << endl;
				// Add ptcl to storage (robot ID, Point Cloud)
				storage->ptCls[storage->testImgCtr] = std::make_pair( syncListenerR1.m_rID, *(syncListenerR1.m_localPtCl) );
				syncListenerR1.m_localPtCl->clear();
				cout << "-------------" << endl << "Else ROBOT 1+2 has received a total ptCls= " << storage->ptCls.size() << endl;
				// Add keyframe pose in initial robot COS to storage
				storage->localPoses[storage->testImgCtr] = std::make_pair(syncListenerR1.m_rID, syncListenerR1.m_localPose);
				storage->localScales[storage->testImgCtr] = std::make_pair(syncListenerR1.m_rID, syncListenerR1.m_sim3LocalPoseScale);
			}

			// Robot 2 received a nonempty keyframe with enough keypts
			if (!syncListenerR2.m_kFrame.img.empty()  && syncListenerR2.m_kFrame.KPts.size() >= 20 ) {

				cout << "---------------------------------------------" << endl << "ROBOT 2 has received a keyframe" << endl;
				//	cout << "current image testImgCtr = " << storage->testImgCtr << endl;
				if(storage->bootnextfivematches){
					storage->robot_2_nextMaches.push_back(storage->testImgCtr);
					storage->rID_ImgCtr_current[2] = storage->rID_ImgCtr_current[2] + 1; 
				}
				cout << "Robot 2 is now have keyframes # " << storage->rID_ImgCtr_current[2] << endl;
				if (boolSaveTestImgs) {
					// Defining strings to save imgs, files, ...
					convImgName.clear(); convImgName.str("");
					convMatchMatrixNameImg.clear(); convMatchMatrixNameImg.str("");
					convMatchMatrixNameFile.clear(); convMatchMatrixNameFile.str("");
					convImgName << "testImgs/robot" << syncListenerR2.m_rID << "TestImg" << storage->testImgCtr << ".jpg";
					convMatchMatrixNameImg << "testImgs/robot" << syncListenerR2.m_rID << "MatchMatrix" << storage->testImgCtr << ".jpg";
					convMatchMatrixNameFile << "testImgs/robot" << syncListenerR2.m_rID << "MatchMatrix" << storage->testImgCtr << ".yml";
					imgName = convImgName.str();
					matchMatrixNameImg = convMatchMatrixNameImg.str();
					matchMatrixNameFile = convMatchMatrixNameFile.str();
					HelperFcts::saveImage(syncListenerR2.m_kFrame.img, imgName);
				}

				// Compute BOW descriptor of current query image
				storage->bide->compute( syncListenerR2.m_kFrame.img, syncListenerR2.m_kFrame.KPts, syncListenerR2.m_kFrame.bowDescriptor );
				syncListenerR2.m_kFrame.fID = storage->testImgCtr;

				// Add keyframe to storage
				storage->kFrames[storage->testImgCtr] = syncListenerR2.m_kFrame;

				// Add ptcl to storage
				storage->ptCls[storage->testImgCtr] = std::make_pair( syncListenerR2.m_rID, *(syncListenerR2.m_localPtCl) );
				syncListenerR2.m_localPtCl->clear();
				cout << "-------------" << endl << "ROBOT 2+1 has received a total ptCls= " << storage->ptCls.size() << endl;

				// Add keyframe pose in initial robot COS to storage
				storage->localPoses[storage->testImgCtr] = std::make_pair(syncListenerR2.m_rID, syncListenerR2.m_localPose);
				storage->localScales[storage->testImgCtr] = std::make_pair(syncListenerR2.m_rID, syncListenerR2.m_sim3LocalPoseScale);

				// RUN FABMAP ROBOT 2
				//cout << "	Run FABMAP" << endl;
				storage->testBOWDescriptors_allRobots.push_back(syncListenerR2.m_kFrame.bowDescriptor);
				storage->fabmap->compare(syncListenerR2.m_kFrame.bowDescriptor, storage->matches, true);


				if (!boolEndAfterFoundMatch) {

					// SEARCH FOR GOOD MATCHES
					storage->searchForGoodMatch(); // if found a good match resulting from FABMAP storage->boolMatch will be set to true

					if (storage->boolMatch == true) // && storage->matchedKFMapDiff > 0 && storage->matchedKFMapDiff < 15)
					{
						//cout << "	Robot2 found a loop-closure" << endl;
						skipmatch++;
						storage->findTrafoInitialGuess();
						
						//storage->findTrafo(storage->iGMap[storage->testImgCtr].first, storage->iGMap[storage->testImgCtr].first);
						//storage->estimateScale();
						storage->boolMatch = false;
						storage->bootnextfivematches = true;
					}
					if (storage->rID_ImgCtr_current[1] > 4 && storage->rID_ImgCtr_current[2] > 4) {
					storage->findnextmatches();
					}
					if (storage->iGMap.size() > 5) //skipmatch == 1) // && storage->matchedKFMapDiff < 7)
					{
						//storage->findTrafo();
						storage->Apply_Optimization();
						HelperFcts::eigenMatrix4fToROSTrafoMsg( storage->iGMap[storage->testImgCtr].first, trafoROSMsgC1C2 );	// not used

						boolEndAfterFoundMatch = true;
						storage->bootnextfivematches = false;
					}
				}
				cout << "size of iGMap is = " << storage->iGMap.size() << endl;
				// Update Poses of robots and PtCls relative to global world COS
				storage->updatePosesOfRobots();
				storage->updatePtCls();

				Eigen::Matrix4f pCam2Robot2(Eigen::Matrix4f::Identity(4, 4)), pRobot2Cam2(Eigen::Matrix4f::Identity(4, 4));
				HelperFcts::poseStampedROSToMatrix4f(storage->localPoses[storage->testImgCtr].second, pCam2Robot2);
				HelperFcts::eigenMatrix4fToROSTrafoMsg(pCam2Robot2, trafoROSMsgC2R2);
				br2.sendTransform(tf::StampedTransform( trafoROSMsgC2R2, ros::Time::now(), "world", "cam2") );


				boolNewKF = true;
				storage->testImgCtr = storage->testImgCtr + 1; // testImgCtr contains nbr of testing Imgs + query Img
				//	helperLocalPtCl_publisher(storage, 2, pub5, 2);
				cout << "---------------------------------------------" << endl << endl << endl;
			} else if (!syncListenerR2.m_kFrame.img.empty()  && syncListenerR2.m_kFrame.KPts.size() < 20 ) {

				if(storage->bootnextfivematches){
					storage->robot_2_nextMaches.push_back(storage->testImgCtr);
					storage->rID_ImgCtr_current[2] = storage->rID_ImgCtr_current[1] + 2; 
				}
				cout << "Robot 2 is now have keyframes # " << storage->rID_ImgCtr_current[2] << endl;
				// Add ptcl to storage
				storage->ptCls[storage->testImgCtr] = std::make_pair( syncListenerR2.m_rID, *(syncListenerR2.m_localPtCl) );
				syncListenerR2.m_localPtCl->clear();
				cout << "-------------" << endl << "Else ROBOT 2+1 has received a total ptCls= " << storage->ptCls.size() << endl;
				// Add keyframe pose in initial robot COS to storage
				storage->localPoses[storage->testImgCtr] = std::make_pair(syncListenerR2.m_rID, syncListenerR2.m_localPose);
				storage->localScales[storage->testImgCtr] = std::make_pair(syncListenerR2.m_rID, syncListenerR2.m_sim3LocalPoseScale);

			}




			// visualize local ptcl
			if (boolEndAfterFoundMatch == true && boolVisualizePtCl == true && boolVisualizedOnce == false) {
				cout << "---------------------------------------------" << endl;
				cout << "Visualize Local PtCl" << endl;
				
				storage->PointCloud_Fusion();
				// ptCls	<imgCtr#, <robot ID, point cloud>>
				pcl::PointCloud<pcl::PointXYZ> assembledLocalPtClUntransformed;// (new pcl::PointCloud<pcl::PointXYZ>());
				typedef std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > > ptClMap;
				for (ptClMap::iterator it = storage->ptCls.begin(); it != storage->ptCls.end(); ++it) {
					if ( (it->second.first == 2) ) {   // means 2nd robot's
						assembledLocalPtClUntransformed += it->second.second; // collect all the point clouds of 2nd robot here.
					}
				}

				//
				// 		NOTED
				//
				// Untransformed point clouds of robot 1 and robot 2 are in "pTCls"
				// Transformed point clouds are in "ptClsInGlobalWorldCOS"

				helperPublishPtCl(assembledLocalPtClUntransformed, pub3); // simply publish the 2nd robot point clouds without transformation
				helperPublishAssembledLocalPtCl(storage, 2, pub2, 2);  // transformed clouds are in "ptClsInGlobalWorldCOS" of robot 2nd
				helperPublishAssembledLocalPtCl(storage, 1, pub1, 4);	// tranformed clouds of robot 1
				helperPublishAssembledGlobalPtCl(storage, pub4, 3); // merged one

				boolNewKF = false;
				boolVisualizedOnce = true;
				cout << "---------------------------------------------" << endl << endl << endl;
			}

			// clear central storage
			bool boolClear = false;

			if ( ( ((storage->testImgCtr % 60) == 0) || ((storage->testImgCtr % 60) == 1) ) && boolClear == true && (storage->testImgCtr != 0) && (storage->testImgCtr != 1) ) {
				cout << "---------------------------------------------" << endl;
				cout << "Clear Storage Data" << endl;
				storage->clearData();
				cout << "---------------------------------------------" << endl << endl << endl;
			}


			//HelperFcts::poseROSToTrafoROSMsg( syncListenerR2.m_localPose.pose, trafoROSMsgR2FromSyncListener);
			//helperPoseToROSTrafoMsg(syncListenerR2.m_localPose.pose,trafoROSMsgR2);
			//br1.sendTransform(tf::StampedTransform( trafoROSMsgC2C1, ros::Time::now(), "world", "cam2") );
			//br2.sendTransform(tf::StampedTransform( trafoROSMsgC1C2, ros::Time::now(), "world", "cam1") );
			//br.sendTransform(tf::StampedTransform(trafoROSMsgR2FromSyncListener, ros::Time::now(), "cam2", "world"));

			if (!storage->testBOWDescriptors_allRobots.empty()) {
				helperDispOutput(storage, true, "testImgs/robot1MatchMatrixEnd.jpg", "testImgs/robot1MatchMatrixFileEnd.yml");
			}

			boolNewKF = false;
			rTest.sleep();
			syncListenerR1.clearData();
			syncListenerR2.clearData();
			cout << endl << endl << endl;

		} // END if (ros::ok())
	} // END if(doTesting)

	/*
		if(doTesting) {
			Ptr<of2::FabMap> fabmap2;
			vector<of2::IMatch> matches2;
			fabmap2 = new of2::FabMap2(storage->clTree, 0.4, 0, of2::FabMap::SAMPLED | of2::FabMap::CHOW_LIU);
			fabmap2->addTraining(storage->trainBOWDescriptors_allRobots);
			CentralStorage* storage2 =  new CentralStorage(2);
			storage2->testBOWDescriptors_allRobots = storage->testBOWDescriptors_allRobots;
			storage2->kFrames = storage->kFrames;
			fabmap2->compare(storage2->testBOWDescriptors_allRobots, storage2->matches, true);
			cout << endl<<endl<< "FABMAP FINAL: " << endl;
			helperDispOutput(storage2,true,"testImgs/robot1MatchMatrixEnd.jpg","testImgs/robot1MatchMatrixFileEnd.yml");
			waitKey(100);
		}
	*/
	storage->~CentralStorage();
	cout << "END OF PROGRAM --------------------------------" << endl;
}