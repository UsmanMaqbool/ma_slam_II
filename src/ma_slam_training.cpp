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


	storage->~CentralStorage();
	cout << "END OF PROGRAM --------------------------------" << endl;
}
