#include "PoseGraph.h"

using namespace std;

Node::Node() : x(0.0), y(0.0), z(0.0),
		ax(0.0), ay(0.0), az(0.0),
		assocKFNbr(0) {
	parent = NULL;
}

Node::~Node() {}

Node * Node::root() {
	Node *cur = this;
	while(cur!=cur->parent) {
		cur=cur->parent;
	}
	this->parent = cur; //do path compression
	return cur;
}



Edge::Edge() {
	// quat.x() = trafoFloat[0]; quat.y() = trafoFloat[1]; quat.z() = trafoFloat[2]; quat.w() = trafoFloat[3];
	// transl.x() = trafoFloat[4]; transl.y() = trafoFloat[5]; transl.z() = trafoFloat[6];
	for(int i=0; i<7; ++i) {
		simFromTo[i] = 0.0;
		if(i==3) {
			simFromTo[i] = 1.0;
		}
	}
	pNode2Node1 = Eigen::Matrix4f::Identity(4,4);
	scaleNode2Node1 = 1.0;

	from = NULL;
	to = NULL;
}

Edge::~Edge() {}



PoseGraph::PoseGraph() : nodes(), edges() {}

PoseGraph::~PoseGraph() {}

void PoseGraph::clear() {
	nodes.clear();
	edges.clear();
}



