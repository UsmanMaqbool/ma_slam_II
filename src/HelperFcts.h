#ifndef HELPERFCTS_H
#define HELPERFCTS_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>



using namespace std;
using namespace cv;

class HelperFcts {

public:

	HelperFcts();
	~HelperFcts();
	static void print4x4Matrix (const Eigen::Matrix4d & matrix);
	static void calc3DPt(Point2f pt2D, float fx, float fy, float cx, float cy, Mat idepth, Eigen::Vector3f & pt3D);
	static void displayImage(string windowName, Mat img);
	static void displayImageKPts(string windowName, Mat img, vector<KeyPoint> kPts);
	static void displayMatches(string windowName, Mat img1, Mat img2, vector<KeyPoint> kPts1, vector<KeyPoint> kPts2, vector<DMatch> matches1to2, bool boolSaveImg);
	static void displaySaveImage(string windowName, Mat img, string nameInclingDir);
	static void eigenMatrix4fToROSTrafoMsg(Eigen::Matrix4f trafo, tf::Transform & trafoROSMsg);
	static void eigenQuatTranslPoseToROSPose(Eigen::Quaternion<float> quat, Eigen::Translation<float,3> transl, geometry_msgs::PoseStamped & pose);//???
	static void invQuatTranslScale(Eigen::Quaternion<float> quat, Eigen::Quaternion<float> & quatInv, Eigen::Translation<float,3> transl, Eigen::Translation<float,3> & translInv, float sc, float & scInv);
	static void invTrafo(Eigen::Matrix4f trafo, Eigen::Matrix4f & trafoInv);
	static void poseStampedROSToMatrix4f(geometry_msgs::PoseStamped poseROS, Eigen::Matrix4f & poseMat);
	static void localPoseTog2o(geometry_msgs::PoseStamped poseROS, int queryImgNbr, string & quat_trans);
	static void localEdgeTog2o(geometry_msgs::PoseStamped poseROS, int queryImgNbr, int testImgNbr, string & quat_trans);
	static void poseROSToTrafoROSMsg(geometry_msgs::Pose trafo,tf::Transform & trafoROSMsg);
	static void PoseToeigenQuatTransl(Eigen::Matrix4f & poseMat, string & quat_trans);
	static void PoseToigSize(Eigen::Matrix4f & poseMat, int & ig_size);
	static void customRansac(vector<Point2f> matchedPtsQuery, vector<Point2f> matchedPtsTest, int dimModel, int maxIter, float threshDist, float inlRatio);
	static void saveImage(Mat img, string nameInclingDir);
	static void saveMatrix(Mat matrix, string nameInclingDir);
	static void saveStringToFile(string content, string nameInclingDir);
	static void Makeg2oFile(vector<string> content, string nameInclingDir);
	static void trafoFloat7Matrix4f(float trafoFloat[7], Eigen::Matrix4f & trafoMat);
	static void trafoFloat7ToQuatTranslScale(float simTrafo[7], Eigen::Quaternion<float> & quat, Eigen::Translation<float,3> & transl, float & sc);
	static void transform3DPt(float simTrafo[7], Eigen::Vector3f pt3D, Eigen::Vector3f & pt3DTransformed);
	static void transform3DPt(Eigen::Matrix4f simTrafo, Eigen::Vector3f pt3D, Eigen::Vector3f & pt3DTransformed);
	static void poses2edge(Eigen::Matrix4f & PrePose, Eigen::Matrix4f & NewPose, Eigen::Matrix4f & poseEdge);
	static void pose2pose(Eigen::Matrix4f & PrePose, Eigen::Matrix4f & poseEdge, Eigen::Matrix4f & NewPose);

};










#endif

