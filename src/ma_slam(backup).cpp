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
// #include <opencv2/nonfree/features2d.hpp>
//#include "opencv2/xfeatures2d.hpp"

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
	std::map<int, int> skipCtr;
	for (int i = 1; i <= storage->nbrRobots; ++i) {
		skipCtr[i] = 0;
	}
	typedef std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > > ptClMap;
	for (ptClMap::iterator it = storage->ptClsInGlobalWorldCOS.begin(); it != storage->ptClsInGlobalWorldCOS.end(); ++it) {
		bool lessThanN = false;
		for (int i = 1; i <= storage->nbrRobots; ++i) {
			if (skipCtr[i] < skipFirstN) {
				lessThanN = true;
			}
		}
		if (lessThanN == false) {
			assembledGlobalPtCl += it->second.second;
		}
		skipCtr[it->second.first] = skipCtr[it->second.first] + 1;
	}
	cout << "globptcl: " << assembledGlobalPtCl.width << endl;
	assembledGlobalPtCl.header.frame_id = "world";
	assembledGlobalPtCl.header.stamp = 0;
	pub.publish(assembledGlobalPtCl);
	//}
}

void helperPublishAssembledLocalPtCl(CentralStorage * storage, int rID, ros::Publisher & pub, int skipFirstN) {

	//if(storage->ptClsInGlobalWorldCOSUpdated[rID] == false) {
	//	cout << "PtCl of robot " << rID << " up-to-date - no visualization" << endl;
	//}
	//else {
	cout << "	Publishing assembled local PtCl of Robot with robot ID: " << rID << endl;

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
	bool boolEndAfterFound1Match = false; // enable to stop looking for further loop closures after the first match - just for verification of the code
	bool boolVisualizePtCl = true; // enable to visualize the local ptcls of the different robots in RVIZ
	bool boolVisualizedOnce = false;
	int nbrRobots = 2; // Number of robots in the system - aim to be set during runtime
	int nbrVisWords = 700;
	int nbrTrainImgs = 300;

	// for the syncListner loop
	SyncListener *syncListenerRobot[nbrRobots];
	string Robot_Sync, Robot_Caminfo, Robot_slave, Robot_PointCloud2Filtered, Robot_kFMsgStamped, Robot_kFGraphMsgStamped;


	ros::init(argc, argv, "ma_slam");
	ros::NodeHandle nh;

	CentralStorage* storage = new CentralStorage(nbrRobots, nbrVisWords);

	cout << "line 332 - storage created" << endl;


	/// ----------------------------------------
	/// ----------------------------------------
	///
	/// TRAINING -------------------------------
	///
	/// ----------------------------------------
	/// ----------------------------------------
	if (doTraining) {

		//	KeyFrameListenerTraining kFListenerTrainingR1(1, nbrTrainImgs, "/usb_cam_training/image_rect", nh, storage);
		//	KeyFrameListenerTraining kFListenerTrainingR2(2, nbrTrainImgs, "/usb_cam_training_robot2/image_raw", nh, storage);
		KeyFrameListenerTraining kFListenerTrainingR1(1, nbrTrainImgs, "/narrow_stereo/image_rect", nh, storage);
		KeyFrameListenerTraining kFListenerTrainingR2(2, nbrTrainImgs, "/narrow_stereo/image_raw", nh, storage);

		// spinning
		cout << endl << "TRAINING started--------------------------------" << endl;


		ros::Rate rTrain(20);
		while (storage->stopDataCollection == false && ros::ok())
		{
			ros::spinOnce();
			rTrain.sleep();
		}


		/// ---- Generate vocabulary (visual words) ---------------------------------------------------------
		if (storage->stopDataCollection == true)
		{
			cout << endl << "store all Training Data in instance of CentralStorage" << endl;
			helperStoreTrainData(storage, kFListenerTrainingR1, kFListenerTrainingR2);

			cout << "Generate Vocabulary" << endl;
			storage->generateVocabulary();

			cout << "Set Vocabulary for BOW Descriptor extractor" << endl;
			(storage->bide)->setVocabulary(storage->vocabulary);

			cout << "Compute BOW Descriptors for Training Images" << endl;
			storage->computeTrainBOWDescriptors();

			cout << "Compute Chow Liu Tree based on Training Data" << endl;
			storage->generateCLTree();

			cout << "save all necessary Training Data on harddisk" << endl;
			helperSaveTrainData(storage);

			cout << "Initialize FabMap instance" << endl << endl;
			storage->fabmap = new of2::FabMap2(storage->clTree, 0.2, 0, of2::FabMap::SAMPLED | of2::FabMap::CHOW_LIU);
			storage->fabmap->addTraining(storage->trainBOWDescriptors_allRobots);
		}
	}
	else {
		storage->stopDataCollection = true;

		helperReadTrainData(storage);

		cout << "Set Vocabulary for BOW Descriptor extractor" << endl;
		(storage->bide)->setVocabulary(storage->vocabulary);

		cout << "Initialize FabMap instance" << endl << endl;
		storage->fabmap = new of2::FabMap2(storage->clTree, 0.4, 0, of2::FabMap::SAMPLED | of2::FabMap::CHOW_LIU);
		storage->fabmap->addTraining(storage->trainBOWDescriptors_allRobots);

	}
	cout << "TRAINING OR LOADING finished --------------------------------" << endl << endl;





	/// ----------------------------------------
	/// ----------------------------------------
	///
	/// TESTING --------------------------------
	///
	/// ----------------------------------------
	/// ----------------------------------------
	if (doTesting) {

		cout << "Initialize synchronized ROS topic listener" << endl;


		for ( int i = 0; i < nbrRobots; i++) {
			Robot_Sync = "syncListenerR" + std::to_string(i);

			Robot_Caminfo = "/usb_cam_r" + std::to_string(i);
			Robot_Caminfo += "/camera_info";

			Robot_slave = "/slave_robot" + std::to_string(i);

			Robot_PointCloud2Filtered = Robot_slave;
			Robot_PointCloud2Filtered += "/pointcloud2Filtered";

			Robot_kFMsgStamped = Robot_slave;
			Robot_kFMsgStamped += "/kFMsgStamped";

			Robot_kFGraphMsgStamped = Robot_slave;
			Robot_kFGraphMsgStamped	+= "/kFGraphMsgStamped";


			syncListenerRobot[i] = new SyncListener (i, Robot_Caminfo, Robot_PointCloud2Filtered, Robot_kFMsgStamped, Robot_kFGraphMsgStamped, nh, storage);
			cout << "---------------------------------------------" << endl << "syncListenerRobot " << i << " created" << endl;

		}


		/*		syncListenerRobot[i]

				SyncListener syncListenerRobot[1](1, "/usb_cam_r1/camera_info",
				                            "/slave_robot1/pointcloud2Filtered",
				                            "/slave_robot1/kFMsgStamped",
				                            "/slave_robot1/kFGraphMsgStamped",
				                            nh, storage);
				SyncListener syncListenerRobot[2](2, "/usb_cam_r2/camera_info",
				                            "/slave_robot2/pointcloud2Filtered",
				                            "/slave_robot2/kFMsgStamped",
				                            "/slave_robot2/kFGraphMsgStamped",
				                            nh, storage);


				SyncListener syncListenerR1(1, "/slave_robot1/keyFrame",
											"/usb_cam_r1/camera_info",
											"/slave_robot1/pointcloud2Filtered",
											"/lsd_slam_r1/pose",
											"/slave_robot1/similarityTransformation",
											nh, storage);


		*/


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
		ros::Rate rTest(5); // adequate spinning frequency???

		while (ros::ok() && storage->testImgCtr < 2000 ) {

			ros::spinOnce();



			for ( int i = 0; i < nbrRobots; i++) {


				//		Mat image = cv::imread(syncListenerRobot[i]-> m_kFrame.img);
				// create image window named "My Image"
				//	namedWindow("My Image");
				// show the image on window
				//	imshow("My Image", image);

//cout << "0 TESTING  --" << 	syncListenerRobot[0]->m_kFrame.KPts.size() << endl << endl;
//cout << "1 TESTING 1--" << 	syncListenerRobot[1]->m_kFrame.KPts.size()  << endl << endl;





				// Robot 1 received a nonempty keyframe with enough keypts
				if (!syncListenerRobot[i]-> m_kFrame.img.empty()) {


					if (boolSaveTestImgs && syncListenerRobot[i]->m_kFrame.KPts.size() >= 15) {
						// Defining strings to save imgs, files, ...

						// Clearing all the string variables which actually save the keyframe images
						convImgName.clear(); convImgName.str("");							// for image name
						convMatchMatrixNameImg.clear(); convMatchMatrixNameImg.str("");		// for confusion matrix
						convMatchMatrixNameFile.clear(); convMatchMatrixNameFile.str("");	// for generating the yml file

						// Generating the image Names
										//robot ID TestImg ImageID (autogenerated)
						convImgName << "testImgs/robot" << syncListenerRobot[i]->m_rID << "TestImg" << storage->testImgCtr << ".jpg";
						convMatchMatrixNameImg << "testImgs/robot" << syncListenerRobot[i]->m_rID << "MatchMatrix" << storage->testImgCtr << ".jpg";
						convMatchMatrixNameFile << "testImgs/robot" << syncListenerRobot[i]->m_rID << "MatchMatrix" << storage->testImgCtr << ".yml";
						cout << "---------------------------------------------" << endl << "ROBOT " << i << " created the jpg image and yml file" << endl;


						// concatination the names
						imgName = convImgName.str();
						matchMatrixNameImg = convMatchMatrixNameImg.str();
						matchMatrixNameFile = convMatchMatrixNameFile.str();
						HelperFcts::saveImage(syncListenerRobot[i]->m_kFrame.img, imgName);    //save the iamge to directory
					}


					// Compute BOW descriptor of current query image
					syncListenerRobot[i]->m_kFrame.fID = storage->testImgCtr;
					storage->bide->compute(syncListenerRobot[i]->m_kFrame.img, syncListenerRobot[i]->m_kFrame.KPts, syncListenerRobot[i]->m_kFrame.bowDescriptor);
					cout << "---------------------------------------------" << endl << "ROBOT " << i << " has Compute BOW descriptor of current query image" << endl;

					// Add Keyframe to storage
					storage->kFrames[storage->testImgCtr] = syncListenerRobot[i]->m_kFrame;
					cout << "---------------------------------------------" << endl << "ROBOT " << i << " Added Keyframe to storage" << endl;

					// Add ptcl to storage
					storage->ptCls[storage->testImgCtr] = std::make_pair( syncListenerRobot[i]->m_rID, *(syncListenerRobot[i]->m_localPtCl) );
					syncListenerRobot[i]->m_localPtCl->clear();
					cout << "---------------------------------------------" << endl << "ROBOT " << i << " has Added ptcl to storage" << endl;

					// Add keyframe pose in initial robot COS to storage
					storage->localPoses[storage->testImgCtr] = std::make_pair(syncListenerRobot[i]->m_rID, syncListenerRobot[i]->m_localPose);
					storage->localScales[storage->testImgCtr] = std::make_pair(syncListenerRobot[i]->m_rID, syncListenerRobot[i]->m_sim3LocalPoseScale);
					cout << "---------------------------------------------" << endl << "ROBOT " << i << " has Add keyframe pose in initial robot COS to storage" << endl;


					if (syncListenerRobot[i]->m_kFrame.KPts.size() >= 15) {
					// RUN FABMAP for each ROBOT
					cout << "	Run FABMAP" << endl;

					//push_back add elements at the end
					// testBOWDescriptors_allRobots is 'MAT'
					storage->testBOWDescriptors_allRobots.push_back(syncListenerRobot[i]->m_kFrame.bowDescriptor);

					//matches – a vector of image match probabilities
					storage->fabmap->compare(syncListenerRobot[i]->m_kFrame.bowDescriptor, storage->matches, true);

					if (!boolEndAfterFound1Match) {

						// SEARCH FOR BEST MATCH
						storage->searchForGoodMatch();
						// Compute transformation between matched Images and subsequently between PtCls
						if (storage->boolMatch == true) {
							cout << "	Robot " << i << " found a loop-closure" << endl;

							skipmatch++;
							storage->findTrafoInitialGuess();
							//storage->findTrafo(storage->iGMap[storage->testImgCtr].first, storage->iGMap[storage->testImgCtr].first)
							//storage->estimateScale();

							if (skipmatch == 1) {
								HelperFcts::eigenMatrix4fToROSTrafoMsg( storage->iGMap[storage->testImgCtr].first, trafoROSMsgC2C1 );
								boolEndAfterFound1Match = true;
							}
							storage->boolMatch = false;
						}
					}
					storage->updatePosesOfRobots();
					storage->updatePtCls();



					Eigen::Matrix4f pCam1Robot(Eigen::Matrix4f::Identity(4, 4)), pRobot1Cam1(Eigen::Matrix4f::Identity(4, 4));
					HelperFcts::poseStampedROSToMatrix4f(storage->localPoses[storage->testImgCtr].second, pCam1Robot);   //pose to matrix 4x4
					HelperFcts::eigenMatrix4fToROSTrafoMsg(pCam1Robot, trafoROSMsgC1R1);
					br1.sendTransform(tf::StampedTransform( trafoROSMsgC1R1, ros::Time::now(), "world", "cam1") );


					boolNewKF = true;
					storage->testImgCtr = storage->testImgCtr + 1;
					cout << "---------------------------------------------" << endl << endl << endl;
				}

			}
		}

		//		cout << "---------------------------------------------" << endl << "We are doing great Sir :)" << endl;













		// visualize local ptcl
		if (boolEndAfterFound1Match == true && boolVisualizePtCl == true && boolVisualizedOnce == false) {
			cout << "---------------------------------------------" << endl;
			cout << "Visualize Local PtCl" << endl;



			pcl::PointCloud<pcl::PointXYZ> assembledLocalPtClUntransformed;// (new pcl::PointCloud<pcl::PointXYZ>());
			typedef std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > > ptClMap;
			for (ptClMap::iterator it = storage->ptCls.begin(); it != storage->ptCls.end(); ++it) {
				if ( (it->second.first == 2) ) {
					assembledLocalPtClUntransformed += it->second.second;
				}
			}

			helperPublishPtCl(assembledLocalPtClUntransformed, pub3);
			helperPublishAssembledLocalPtCl(storage, 2, pub2, 2);
			helperPublishAssembledLocalPtCl(storage, 1, pub1, 4);
			helperPublishAssembledGlobalPtCl(storage, pub4, 3);

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


		//HelperFcts::poseROSToTrafoROSMsg( syncListenerRobot[2]->m_localPose.pose, trafoROSMsgR2FromSyncListener);
		//helperPoseToROSTrafoMsg(syncListenerRobot[2]->m_localPose.pose,trafoROSMsgR2);
		//br1.sendTransform(tf::StampedTransform( trafoROSMsgC2C1, ros::Time::now(), "world", "cam2") );
		//br2.sendTransform(tf::StampedTransform( trafoROSMsgC1C2, ros::Time::now(), "world", "cam1") );
		//br.sendTransform(tf::StampedTransform(trafoROSMsgR2FromSyncListener, ros::Time::now(), "cam2", "world"));

		if (!storage->testBOWDescriptors_allRobots.empty()) {
			helperDispOutput(storage, true, "testImgs/robot1MatchMatrixEnd.jpg", "testImgs/robot1MatchMatrixFileEnd.yml");
		}

		boolNewKF = false;
		rTest.sleep();
		syncListenerRobot[1]->clearData();
		syncListenerRobot[2]->clearData();
		cout << endl << endl << endl;



		* /

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
