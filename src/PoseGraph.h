#ifndef POSEGRAPH_H
#define POSEGRAPH_H

#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


using namespace std;

class Node {
public:
	// quat.x() = trafoFloat[0]; quat.y() = trafoFloat[1]; quat.z() = trafoFloat[2]; quat.w() = trafoFloat[3];
	// transl.x() = trafoFloat[4]; transl.y() = trafoFloat[5]; transl.z() = trafoFloat[6];
	float camToOrigin[7];
	Eigen::Matrix4f pNodeOrigin;
	float scale;
	float x, y, z; // position of Node relative to origin
	float ax, ay, az; //heading (euler angles around x,y,z in initial frame (initial robot frame for local pose graph of robot; world frame for global pose graph)

	int assocKFNbr; // associ KFNbr = storage->testImgCtr

	Node *parent; //for graph connectivity

	Node();
	~Node();
	Node * root();
};

class Edge {
public:
	// quat.x() = trafoFloat[0]; quat.y() = trafoFloat[1]; quat.z() = trafoFloat[2]; quat.w() = trafoFloat[3];
	// transl.x() = trafoFloat[4]; transl.y() = trafoFloat[5]; transl.z() = trafoFloat[6];
	float simFromTo[7];
	Eigen::Matrix4f pNode2Node1; // pose of Node2 ("To") relative to Node1 ("From")
	float scaleNode2Node1;

	Node *from, *to;

	Edge();
	~Edge();
	//cv::Scalar color;
};





class PoseGraph {
public:
	std::map<int, Node> nodes; //maps UIDs to nodes
	std::map<int, Edge> edges;


	PoseGraph();// { lastOptimize=0; lastCluster = 0; numClusters = 0; }
	~PoseGraph();

	void clear();


	//	cv::Mat nnGraphPoints; //stores points as n-dimensional vectors in a matrix for NN and clustering
//	std::map<int, int> nnIndices; //maps nnGraphPoints indices to node UIDs
//	std::map<int, int> reverseNNIndices; //what it says

//	std::vector<RGBImageWithKeypoints> imageData;
//	std::vector<opengv::points_t> featurePoints;
//	std::map<int, int> imgToNodeIdx;

//	bool AddNode(const visualization_msgs::Marker &m, int robot);

	//bool AddEdge(const visualization_msgs::Marker &m, int robot, int uid1, int uid2, cv::Scalar color);
//	bool AddEdge(const visualization_msgs::Marker &m, int robot, int uid1, int uid2);
//	bool AddEdge(int uid1, int uid2); //adds a new edge with id edges.size() and uniform color, mostly for quick visualization

//	void UpdatePose(const visualization_msgs::Marker &m, int robot);
//	void UpdatePose(int uid, float x, float y, float z, float ax, float ay, float az);
//	bool UpdateEdge(const visualization_msgs::Marker &m, int robot);
//	bool UpdateEdge(int uid);

	//cv::Mat Display(cv::Mat img,  std::vector<float> origX, std::vector<float> origY, float resolution);
	//cv::Mat Display(cv::Mat img,  std::vector<float> origX, std::vector<float> origY, float resolution, cv::Mat &labels);
	//cv::Mat Display(cv::Mat img, float origX, float origY, float resolution);

	//void ExtractSubgraphOffsets(std::vector<nav_msgs::OccupancyGrid> &subMaps,
	//			     std::vector<int> &offsetX, std::vector<int> &offsetY,
	//			     int &sizeX, int &sizeY,
	//			     std::vector<bool> include=std::vector<bool>()); //for drawing and messages
	//void ToMsg(visualization_msgs::MarkerArray &msg,
	//	   std::vector<nav_msgs::OccupancyGrid> &subMaps,
	//	   std::vector<bool> include=std::vector<bool>(), float originShift=1.0f); //converts the subset of the graph specified by "include"
	//void FromMsg(const visualization_msgs::MarkerArray &msg);


	//int SaveToDisk(std::vector<nav_msgs::OccupancyGrid> &subMaps, const std::string fileName = "graph.dat");
	//int LoadFromDisk(std::vector<nav_msgs::OccupancyGrid> &subMaps, const std::string fileName = "graph.dat");

};















#endif
