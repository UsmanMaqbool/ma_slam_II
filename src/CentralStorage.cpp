#include "CentralStorage.h"
// #include <pcl/visualization/cloud_viewer.h>
#include "ceres/ceres.h"
#include "ceres/types.h"
#include "ceres/read_g2o.h"
#include "ceres/pose_graph_3d_error_term.h"
using namespace std;
using namespace opengv;

namespace ceres
{
namespace examples
{

// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void BuildOptimizationProblem(const VectorOfConstraints &constraints, MapOfPoses *poses, ceres::Problem *problem)
{
	CHECK(poses != NULL);
	CHECK(problem != NULL);
	if (constraints.empty())
	{
		LOG(INFO) << "No constraints, no problem to optimize.";
		return;
	}

	ceres::LossFunction *loss_function = NULL;
	ceres::LocalParameterization *quaternion_local_parameterization = new EigenQuaternionParameterization;

	for (VectorOfConstraints::const_iterator constraints_iter = constraints.begin();
		 constraints_iter != constraints.end(); ++constraints_iter)
	{
		const Constraint3d &constraint = *constraints_iter;

		MapOfPoses::iterator pose_begin_iter = poses->find(constraint.id_begin);
		CHECK(pose_begin_iter != poses->end()) << "Pose with ID: " << constraint.id_begin << " not found.";
		MapOfPoses::iterator pose_end_iter = poses->find(constraint.id_end);
		CHECK(pose_end_iter != poses->end()) << "Pose with ID: " << constraint.id_end << " not found.";

		const Eigen::Matrix<double, 6, 6> sqrt_information = constraint.information.llt().matrixL();
		// Ceres will take ownership of the pointer.
		ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);

		problem->AddResidualBlock(cost_function, loss_function, pose_begin_iter->second.p.data(),
								  pose_begin_iter->second.q.coeffs().data(), pose_end_iter->second.p.data(),
								  pose_end_iter->second.q.coeffs().data());

		problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(), quaternion_local_parameterization);
		problem->SetParameterization(pose_end_iter->second.q.coeffs().data(), quaternion_local_parameterization);
	}

	// The pose graph optimization problem has six DOFs that are not fully
	// constrained. This is typically referred to as gauge freedom. You can apply
	// a rigid body transformation to all the nodes and the optimization problem
	// will still have the exact same cost. The Levenberg-Marquardt algorithm has
	// internal damping which mitigates this issue, but it is better to properly
	// constrain the gauge freedom. This can be done by setting one of the poses
	// as constant so the optimizer cannot change it.
	MapOfPoses::iterator pose_start_iter = poses->begin();
	CHECK(pose_start_iter != poses->end()) << "There are no poses.";
	problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
	problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem *problem)
{
	CHECK(problem != NULL);

	ceres::Solver::Options options;
	options.max_num_iterations = 200;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

	ceres::Solver::Summary summary;
	ceres::Solve(options, problem, &summary);

	std::cout << summary.FullReport() << '\n';

	return summary.IsSolutionUsable();
}

// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
bool OutputPoses(const std::string &filename, const MapOfPoses &poses)
{
	std::fstream outfile;
	outfile.open(filename.c_str(), std::istream::out);
	if (!outfile)
	{
		LOG(ERROR) << "Error opening the file: " << filename;
		return false;
	}
	for (std::map<int, Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Pose3d>>>::const_iterator
			 poses_iter = poses.begin();
		 poses_iter != poses.end(); ++poses_iter)
	{

		const std::map<int, Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Pose3d>>>::value_type &
			pair = *poses_iter;
		outfile << pair.first << " " << pair.second.p.transpose() << " " << pair.second.q.x() << " " << pair.second.q.y()
				<< " " << pair.second.q.z() << " " << pair.second.q.w() << '\n';
	}
	return true;
}

// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
bool Final_Pose(const MapOfPoses &poses, std::map<int, Eigen::Matrix4f> &PosesinWorld)
{

	Eigen::Matrix4f Mat_4f;
	for (std::map<int, Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Pose3d>>>::const_iterator
			 poses_iter = poses.begin();
		 poses_iter != poses.end(); ++poses_iter)
	{

		const std::map<int, Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Pose3d>>>::value_type &
			pair = *poses_iter;

		Eigen::Quaternion<float> quat;
		quat.w() = pair.second.q.w();
		quat.x() = pair.second.q.x();
		quat.y() = pair.second.q.y();
		quat.z() = pair.second.q.z();

		Mat_4f.block(0, 0, 3, 3) = quat.toRotationMatrix();
		Mat_4f.block(0, 3, 3, 1) << pair.second.p.x(), pair.second.p.y(), pair.second.p.z();

		PosesinWorld.insert(std::pair<int, Eigen::Matrix4f>(pair.first, Mat_4f));
	}
	return true;
}

} // namespace examples
} // namespace ceres

// Constructor
CentralStorage::CentralStorage(int nbrRobotsInput, int nbrVisWordsInput)
	: nbrRobots(nbrRobotsInput), nbrVisWords(nbrVisWordsInput), testImgCtr(0), stopDataCollection(false)
{
	//	// Init camera Matrix and distortion coefficients
	//	//float camMatArr[3][3] = {{442.777026, 0, 291.591624},{0, 444.581889, 207.395871},{0, 0, 1}};
	//	//float distCoeffsArr[5] = {-0.343887, 0.100088, -0.001316, -0.000163, 0};
	//	//camMat = Mat(3,3,CV_32FC1,camMatArr);
	//	//distCoeffs = Mat(5,1,CV_32FC1,distCoeffsArr);

	// Detector, Extractor, Matcher, BOW Img Descriptor, BOW KMeans trainer
	// detector = new DynamicAdaptedFeatureDetector(AdjusterAdapter::create("SURF"),150,250,4); //
	// detector = new DynamicAdaptedFeatureDetector(AdjusterAdapter::create("SIFT"),400,500,5); //
	// detector = new SiftFeatureDetector(0, 1, 1, 5, 0.9); // nfeatures, noctavelayers,contrastthreshold (higher->less
	// features), edgethreshold (higher->more features) ,sigma
	detector = new SiftFeatureDetector(0, 4, 0.04, 10, 1.0); // nfeatures, noctavelayers,contrastthreshold (higher->less
															 // features), edgethreshold (higher->more features) ,sigma

	// detector = new SiftFeatureDetector(0, 3, 0.15, 5, 0.9);0, 3, 0.15, 5, 0.9

	// extractor = new SurfDescriptorExtractor(500,4,2,false,true); //hessian threshold,
	// noctave,noctavelayers,extended,upright
	extractor = new SiftDescriptorExtractor();		   // hessian threshold, noctave,noctavelayers,extended,upright
	matcher = DescriptorMatcher::create("FlannBased"); // Fast Library for Approximate Nearest Neighbors
	bide = new BOWImgDescriptorExtractor(extractor, matcher);
	// clustering (nbr clusters = nbr vis words)
	// define Term Criteria
	TermCriteria tc(CV_TERMCRIT_ITER, 100, 0.001);
	// retries number
	int retries = 1;
	// necessary flags
	int flags = KMEANS_PP_CENTERS;
	BOWTrainer = new BOWKMeansTrainer(nbrVisWords, tc, retries, flags);

	// Initializing Kalman Filter for scale estimation
	KalmanFilterScale::Vector x(1);
	x(1) = 1.0;
	static const float _P0[] = {10.0};
	KalmanFilterScale::Matrix P0(1, 1, _P0);
	filter.init(x, P0);

	// Initialize global Transforms with identity transform and initialize ptClsUpdated to true
	for (int i = 1; i <= nbrRobots; ++i)
	{
		pRobotsWorld[i] = Eigen::Matrix4f::Identity(4, 4);
		ptClsInGlobalWorldCOSUpdated[i] = false;
		robotScales[i] = 1.0;
	}
	this->qnode_no = 0; // initializing the nodes
	this->tnode_no = 0; // initializing the nodes
}

CentralStorage::~CentralStorage()
{
}

void CentralStorage::generateVocabulary()
{
	this->BOWTrainer->add(this->trainDescriptors_allRobots);
	this->vocabulary =
		this->BOWTrainer->cluster(); // Returns the cluster centers (descriptors are clustered into nbr vis words)
}

void CentralStorage::computeTrainBOWDescriptors()
{
	int imgCtr = 0;
	cv::Mat BOWDescriptors;

	std::vector<cv::Mat>::iterator itImg;
	for (itImg = this->trainImgs_allRobots.begin(); itImg != this->trainImgs_allRobots.end(); ++itImg)
	{
		this->bide->compute(*itImg, this->trainKPts_allRobots[imgCtr], BOWDescriptors);
		this->trainBOWDescriptors_allRobots.push_back(BOWDescriptors);
		imgCtr = imgCtr + 1;
	}
}

void CentralStorage::generateCLTree()
{
	of2::ChowLiuTree treeBuilder;
	treeBuilder.add(this->trainBOWDescriptors_allRobots);
	this->clTree = treeBuilder.make();
}

void CentralStorage::searchForGoodMatch()
{
	// FOR TMR
	//* dont consider first several imsg
	// add consecutive if from other robot
	// no match if other robot has some there
	if (this->testImgCtr > 0)
	{

		ostringstream convStringFileName, convStringContent;
		string matchedImgsFileName, matchedImgsContent;
		int kFrameNbr = -10000;
		int rID = this->kFrames[this->testImgCtr].rID;
		vector<of2::IMatch>::const_iterator it;
		float maxLikeli = 0;
		float matchCurrent(0.0), matchLast(0.0), matchSecondLast(0.0);
		// int nbrImgs = (int) -0.5+(1/4+2*this->matches.size())^(1/2);
		// for last query Img (last keyframe) loop through all (test)imgs except the query img on its own (query img - query
		// img : this->matches.end()-this->testImgCtr-1)
		for (it = this->matches.end() - this->testImgCtr; it != this->matches.end(); ++it)
		{
			// cout<<"this->matches.end()" << this->matches.end() << endl;
			// cout<<"this->matches.end() - this->testImgCtr" << this->matches.end() - this->testImgCtr << endl;
			// cout<<"this->testImgCtr" << this->testImgCtr << endl;
			//			if(this->testImgCtr < 30) { // dont consider matches among the first 6 keyframes
			//			if(this->testImgCtr < 8) { // dont consider matches among the first 6 keyframes
			//	if (this->testImgCtr < 45) { // dont consider matches among the first 6 keyframes
			//		continue;
			//	}

			//			if(it->match < 0.75) {
			if (it->match < 0.80)
			{
				//				cout << "		match probability too low" << endl;
				continue;
			}

			if (it->imgIdx != -1)
			{ // comparing query Img to testing Img associated with kFrameNbr
				kFrameNbr = it->imgIdx;
			}
			else if (it->imgIdx == -1)
			{ // comparing query Img to itself
				//				cout << "		compared to its own2" << endl;
				kFrameNbr = this->kFrames.size() - 1;
				continue; // Do not test to itself
			}

			if (this->kFrames[this->testImgCtr].rID == this->kFrames[kFrameNbr].rID)
			{ // No matches of same robot
				//				cout << "		same robot ID" << endl;
				continue;
			}

			// name of file determines queryImg specifications
			convStringFileName << "matchingImgs/robot" << this->kFrames[this->testImgCtr].rID << "MatchedImgsQueryImg"
							   << this->kFrames[this->testImgCtr].fID << ".yml";
			matchedImgsFileName = convStringFileName.str();

			convStringContent << "robot: " << this->kFrames[kFrameNbr].rID
							  << "   MatchedImg: " << this->kFrames[kFrameNbr].fID << "   MatchProbab: " << it->match
							  << "     "
							  << "Query # " << this->testImgCtr << " , Test # " << kFrameNbr;
			//	cout << "robot: " << this->kFrames[kFrameNbr].rID << "   MatchedImg: " << this->kFrames[kFrameNbr].fID << "
			// MatchProbab: " << it->match << "     " << "Query # " << this->testImgCtr << " , Test # " << kFrameNbr << endl;
			matchedImgsContent = convStringContent.str();
			//	HelperFcts::saveStringToFile(matchedImgsContent, "matchingImgs/robot.yml");
			this->matchedKeyFrames.first = this->testImgCtr; // query Img
			this->matchedKeyFrames.second = kFrameNbr;		 // matched test Img
			//	cout << "matchedKeyFramesMap.size() " << matchedKeyFramesMap.size() ;
			/*

		  Saving the difference between last two matches

	  */
			if (this->matchedKeyFramesMap.size() > 0)
			{

				int QueryImgCtr_diff =
					this->matchedKeyFrames.first -
					this->matchedKeyFramesMap.rbegin()->second.first; // - this->matchedKeyFramesMap.rbegin()->second.first;
																	  // //matched keyframe query number - difference
				int TestImgCtr_diff =
					this->matchedKeyFrames.second -
					this->matchedKeyFramesMap.rbegin()->second.second; // - this->matchedKeyFramesMap.rbegin()->second.second;
																	   // //matched keyframe query number of 2nd robots -
																	   // difference

				//	cout << "QueryImgCtr_diff" << QueryImgCtr_diff << endl;
				//	cout << "TestImgCtr_diff" << TestImgCtr_diff << endl;

				this->matchedKFMapDiff = max(QueryImgCtr_diff, TestImgCtr_diff);
				//	cout << "matchedKFMapDiff =  " << matchedKFMapDiff << endl;
			}

			this->matchedKeyFramesMap.insert(std::pair<int, std::pair<int, int>>(
				this->testImgCtr, this->matchedKeyFrames)); // store with key "nbr of query image (= this->testImgCtr)"
			if (!matchedImgsContent.empty())
			{
				// cout << "		GOOD MATCH" << endl;
				//	HelperFcts::saveStringToFile(matchedImgsContent, matchedImgsFileName);
				this->boolMatch = true; // bool to get into next stage of FindTrafoInitialGuess
			}

			// this->match_Probability.insert (this->testImgCtr, std::pair<double, int>(match_value, this->matchedKeyFrames);
			// // max efficiency inserting

			this->match_Probability[this->testImgCtr] = std::make_pair(it->match, this->matchedKeyFrames.second);
			// cout << "Query IMG -> " << this->testImgCtr << "Test Image, " << this->matchedKeyFrames.second << " the probability = " << it->match << endl;
			//

			if (this->iG_match < it->match)
			{
				this->iG_match = it->match;
				this->iG_match_query = this->testImgCtr;
				this->iG_match_test = this->matchedKeyFrames.second;
			}

		} // END for loop through matches

	} // END if this->testImgCtr > 0
}

/*

*?   es me hum first matching ki queryImgNbr and testImgNbr save karty hain iGMap me
* Find the Least Size Initial guess from the matching

 */

void CentralStorage::findTrafoInitialGuess()
{
	if (this->iGMap.size() == 0)
	{
		cout << "First Match Found" << endl;
		int queryImgNbr = this->testImgCtr; // query Image Number
		int testImgNbr;						// test image number will be assigned later
		testImgNbr = matchedKeyFramesMap[queryImgNbr].second;
		// Findout the Robot ID
		keyFrame iG_Query_KF = this->kFrames[queryImgNbr];
		int Query_rID = iG_Query_KF.rID;
		// cout << "Query robot ID = " << Query_rID << endl;
		// cout << "Query image number = " << queryImgNbr << endl;
		// cout << "Test image number = " << testImgNbr << endl;

		ostringstream matched_infoo;
		matched_infoo << "Query_RID" << setw(10) << "Query #" << setw(15) << "Test #" << setw(19) << "Comments";
		this->Matching_info.push_back(matched_infoo.str());
		matched_infoo.str("");
		matched_infoo.clear();
		matched_infoo << Query_rID << setw(15) << queryImgNbr << setw(15) << testImgNbr << setw(35) << "First Match of iG Map";
		this->Matching_info.push_back(matched_infoo.str());
		Eigen::Matrix4f iG, iGCross;

		cout << endl
			 << "********** First Matching Found **********" << endl;
		this->iG_RobotID = Query_rID;

		this->iGMap[queryImgNbr] =
			std::make_pair(Eigen::Matrix4f::Identity(4, 4), testImgNbr); // Use identity matrix for the iG Map
	}
}

/*
	USED for find out the matching and PUSHING all the edges to the SE3_g2o std vector

*/
void CentralStorage::calculateMatching(int queryImgNbr, int testImgNbr, string &edges3d)
{

	// cout << " now started calculateMatching " << endl;
	keyFrame queryKF, testKF;			  // current query KF and test KF
	queryKF = this->kFrames[queryImgNbr]; // extracting the the query keyframe
	testKF = this->kFrames[testImgNbr];
	Eigen::Matrix4f iG;
	// Extract descriptors from imgs
	Mat queryImgDescr, testImgDescr;
	this->extractor->compute(queryKF.img, queryKF.KPts, queryImgDescr);
	this->extractor->compute(testKF.img, testKF.KPts, testImgDescr);

	//  match descriptors
	std::vector<DMatch> resultingMatches;
	this->matcher->match(queryImgDescr, testImgDescr, resultingMatches);

	//  Filter out bad matches
	double max_dist = 0;
	double min_dist = 100;
	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < queryImgDescr.rows; i++)
	{
		double dist = resultingMatches[i].distance;
		if (dist < min_dist)
			min_dist = dist;
		if (dist > max_dist)
			max_dist = dist;
	}
	std::vector<DMatch> good_matches;
	for (int i = 0; i < queryImgDescr.rows; i++)
	{
		if (resultingMatches[i].distance <= max(3 * min_dist, 10.0))
		{
			good_matches.push_back(resultingMatches[i]);
		}
	}

	string TestMatches_Image = "TestMatch_R(1)" + std::to_string(testImgNbr) + "_R(2)" + std::to_string(queryImgNbr);
	//  calling function to display the matches
	HelperFcts::displayMatches(TestMatches_Image, queryKF.img, testKF.img, queryKF.KPts, testKF.KPts, good_matches,
							   true);

	// derive correspondences based on random point-cloud
	bearingVector_t bearingVectorQuery, bearingVectorTest;
	bearingVectors_t bearingVectorsQuery;
	bearingVectors_t bearingVectorsTest;

	//cout << "in Central storage-> helpfcts -> Good matches size " << good_matches.size() << endl;

	//	HelperFcts::displayImage("queryKF_Image", queryKF.img);
	//	HelperFcts::displayImage("testKF_Image", testKF.img);
	//	HelperFcts::displayImageKPts("queryKF_Image_KP", queryKF.img, queryKF.KPts);
	//	HelperFcts::displayImageKPts("testKF_Image_KP", testKF.img, testKF.KPts);*/

	vector<Point2f> matchedPtsQuery, matchedPtsTest;
	for (int i = 0; i < good_matches.size(); ++i)
	{
		Point2f matchedPtQuery = queryKF.KPts[good_matches[i].queryIdx].pt;
		Point2f matchedPtTest = testKF.KPts[good_matches[i].trainIdx].pt;

		matchedPtsQuery.push_back(matchedPtQuery);
		matchedPtsTest.push_back(matchedPtTest);

		bearingVectorQuery[0] = matchedPtQuery.x / sqrt(matchedPtQuery.x * matchedPtQuery.x + matchedPtQuery.y * matchedPtQuery.y + 1.0);
		bearingVectorQuery[1] = matchedPtQuery.y / sqrt(matchedPtQuery.x * matchedPtQuery.x + matchedPtQuery.y * matchedPtQuery.y + 1.0);
		bearingVectorQuery[2] = 1.0; //sqrt(1.0 - matchedPtQuery.x*matchedPtQuery.x - matchedPtQuery.y*matchedPtQuery.y);
		bearingVectorQuery.normalize();

		bearingVectorTest[0] = matchedPtTest.x / sqrt(matchedPtTest.x * matchedPtTest.x + matchedPtTest.y * matchedPtTest.y + 1.0);
		bearingVectorTest[1] = matchedPtTest.y / sqrt(matchedPtTest.x * matchedPtTest.x + matchedPtTest.y * matchedPtTest.y + 1.0);
		//sqrt(1.0 - matchedPtTest.x*matchedPtTest.x - matchedPtTest.y*matchedPtTest.y)
		bearingVectorTest[2] = 1.0; //sqrt(1.0 - matchedPtTest.x*matchedPtTest.x - matchedPtTest.y*matchedPtTest.y);
		bearingVectorTest.normalize();

		bearingVectorsQuery.push_back(bearingVectorQuery);
		bearingVectorsTest.push_back(bearingVectorTest);
	}
	// create the central relative adapter
	relative_pose::CentralRelativeAdapter adapter(bearingVectorsQuery, bearingVectorsTest);
	// create a RANSAC object
	sac::Ransac<sac_problems::relative_pose::CentralRelativePoseSacProblem> ransac;
	//  create a CentralRelativePoseSacProblem
	//  (set algorithm to STEWENIUS, NISTER, SEVENPT, or EIGHTPT)
	std::shared_ptr<sac_problems::relative_pose::CentralRelativePoseSacProblem>
		relposeproblem_ptr(
			new sac_problems::relative_pose::CentralRelativePoseSacProblem(
				adapter,
				sac_problems::relative_pose::CentralRelativePoseSacProblem::NISTER)); // nine point

	// run ransac
	ransac.sac_model_ = relposeproblem_ptr;
	ransac.threshold_ = 2 * (1.0 - cos(atan(sqrt(2.0) * 0.5 / 800.0)));
	ransac.max_iterations_ = 250;

	ransac.computeModel();

	//  get the result
	transformation_t pCamTestCamQueryOpenGV = ransac.model_coefficients_;

	// Eigen::Matrix4f iG(Eigen::Matrix4f::Identity(4, 4));
	Eigen::Matrix4d iGdouble(Eigen::Matrix4d::Identity(4, 4));
	iGdouble.block(0, 0, 3, 4) = pCamTestCamQueryOpenGV;
	iG = iGdouble.cast<float>();
	// HelperFcts::print4x4Matrix (iG.cast<double>());

	string quat_trans;
	HelperFcts::PoseToeigenQuatTransl(iG, quat_trans);
	//cout << "quat_trans :" << quat_trans << endl;
	//pose_id x y z q_x q_y q_z q_w

	ostringstream edges3d_make;

	double match_information;

	for (auto &x : match_Probability)
	{
		if (x.first == queryImgNbr && x.second.second == testImgNbr)
		{
			match_information = x.second.first;
			//	cout << "Mached Query IMG -> " << x.first << "Test Image, " << x.second.second << " the probability = " << x.second.first << endl;
		}
	}

	// int informationUncertainity = (1-match_information)*100;
	// int informationUncertainity = good_matches.size() * 1;

	ostringstream information_matrix;
	//information_matrix << " " << informationUncertainity << " 0 0 0 0 0 " << informationUncertainity << " 0 0 0 0 " << informationUncertainity << " 0 0 0 " << informationUncertainity << " 0 0 " << informationUncertainity << " 0 " <<   informationUncertainity;
	information_matrix << " "
					   << "1.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000 0.000000 0.000000 0.000000 0.000000 1.000000 0.000000 0.000000 0.000000 1.000000 0.000000 0.000000 1.000000 0.000000 1.000000";

	edges3d = quat_trans + information_matrix.str();

	//cout << "match_edge of " << queryImgNbr << " and " << testImgNbr << " " << edges3d << endl;
}

/**
 *?  es ne next 5 matches find out karny hain
 * 	PREVIOUS Keyframes in iG Map: 
 * 					p_queryImgNbr  and   p_testImgNbr
 * 	Current Keyframes in iG Map:
 * 					robot_1_nextMaches[a] and robot_2_nextMaches[a]
 * 	@Query_rID   = Query Robot ID (for global access: this->iG_RobotID)
 *
 *
 */
void CentralStorage::findnextmatches()
{
	if (this->iGMap.size() != 0)
	{

		int p_queryImgNbr = this->iGMap.rbegin()->first;		// last matched!!!
		int p_testImgNbr = this->iGMap.rbegin()->second.second; // last matched!!!
		// Findout the Robot ID
		keyFrame iG_Query_KF = this->kFrames[p_queryImgNbr];
		int Query_rID = iG_Query_KF.rID;
		Eigen::Matrix4f iG;
		string edges3d;
		for (int a = 0; a < 5; a = a + 1)
		{

			if (Query_rID == 1)
			{
				cout << endl
					 << endl
					 << "----------------------------";
			//	cout << "Robot 1 next five keyframes = " << robot_1_nextMaches[a] << " and Robot 2 next five keyframes = " << robot_2_nextMaches[a] << endl;
				calculateMatching(robot_1_nextMaches[a], robot_2_nextMaches[a], edges3d);
				this->match_edge[robot_1_nextMaches[a]] = std::make_pair(edges3d, robot_2_nextMaches[a]);
			//	cout << "matching_edge size " << this->match_edge.size() << endl;

				this->iGMap[robot_1_nextMaches[a]] = std::make_pair(iG, robot_2_nextMaches[a]); //only true if the Robot1 is Query robot
				//? Cross Matching
				// Top left to next right
				calculateMatching(p_queryImgNbr, robot_2_nextMaches[a], edges3d);
				this->match_edge_leftdownright[p_queryImgNbr] = std::make_pair(edges3d, robot_2_nextMaches[a]);
			//	cout << "matching_edge size " << this->match_edge_leftdownright.size() << endl;
				// Top right to next left
				calculateMatching(robot_1_nextMaches[a], p_testImgNbr, edges3d);
				this->match_edge_rightdownleft[robot_1_nextMaches[a]] = std::make_pair(edges3d, p_testImgNbr);
			//	cout << "matching_edge size " << this->match_edge_rightdownleft.size() << endl;

				ostringstream matched_infoo;
				matched_infoo << Query_rID << setw(15) << robot_1_nextMaches[a] << setw(15) << robot_2_nextMaches[a] << setw(15) << a << "  Match of iG Map (First Case " << this->iG_RobotID;
				this->Matching_info.push_back(matched_infoo.str());

				//? Updating the previous pose values
				p_queryImgNbr = robot_1_nextMaches[a];
				p_testImgNbr = robot_2_nextMaches[a];
			}
			else
			{
				cout << endl
					 << endl
					 << "----------------------------";
			//	cout << "Robot 1 next five keyframes = " << robot_2_nextMaches[a] << " and Robot 2 next five keyframes = " << robot_1_nextMaches[a] << endl;

				calculateMatching(robot_2_nextMaches[a], robot_1_nextMaches[a], edges3d);
				this->iGMap[robot_2_nextMaches[a]] = std::make_pair(iG, robot_1_nextMaches[a]); //only true if the Robot1 is Query robot

				this->match_edge[robot_2_nextMaches[a]] = std::make_pair(edges3d, robot_1_nextMaches[a]);
			//	cout << "matching_edge size " << this->match_edge.size() << endl;

				//? Cross Matching
				// Top left to next right
				calculateMatching(p_queryImgNbr, robot_1_nextMaches[a], edges3d);
				this->match_edge_leftdownright[p_queryImgNbr] = std::make_pair(edges3d, robot_1_nextMaches[a]);
			//	cout << "matching_edge size " << this->match_edge_leftdownright.size() << endl;
				// Top right to next left
				calculateMatching(robot_2_nextMaches[a], p_testImgNbr, edges3d);
				this->match_edge_rightdownleft[robot_2_nextMaches[a]] = std::make_pair(edges3d, p_testImgNbr);
			//	cout << "matching_edge size " << this->match_edge_rightdownleft.size() << endl;
				ostringstream matched_infoo;
				matched_infoo << Query_rID << setw(15) << robot_2_nextMaches[a] << setw(15) << robot_1_nextMaches[a] << setw(15) << a << "  Match of iG Map (2nd Case)";
				this->Matching_info.push_back(matched_infoo.str());

				//? Updating the previous pose values
				p_queryImgNbr = robot_2_nextMaches[a];
				p_testImgNbr = robot_1_nextMaches[a];
			}
			cout << "iGMap size : " << this->iGMap.size() << endl
				 << endl;
		}
	}
}

void CentralStorage::findTrafo()
{
	// 	iGMap[queryImgNbr] = (iG, testImgNbr);
	// `std::map<int,std::pair <int,pcl::PointCloud<pcl::PointXYZ> > >`
	cout << "2: ===========> Entering to find the Trasformation" << endl;
	ostringstream SrcFileName, TarFileName, fiFileName;

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptClToPtrSource(
		new pcl::PointCloud<pcl::PointXYZ>()); // Original point cloud (cloud_icp = ptClToPtrSource )
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptClToPtrTarget(
		new pcl::PointCloud<pcl::PointXYZ>()); // ICP output point cloud (cloud_in = ptClToPtrTarget )

	pcl::PointCloud<pcl::PointXYZ>::Ptr P_ptClToPtrSource(
		new pcl::PointCloud<pcl::PointXYZ>()); // Original point cloud (cloud_icp = ptClToPtrSource )
	pcl::PointCloud<pcl::PointXYZ>::Ptr P_ptClToPtrTarget(
		new pcl::PointCloud<pcl::PointXYZ>()); // ICP output point cloud (cloud_in = ptClToPtrTarget )

	int P_SNbr, P_TNbr;

	typedef std::map<int, std::pair<Eigen::Matrix4f, int>> ImgCtr_copy;
	/**
 	* Direct Matching between the Query and Test Robots Point Clouds
 	* 
 	*/

	for (ImgCtr_copy::iterator it_ImgCtr = this->iGMap.begin(); it_ImgCtr != this->iGMap.end(); ++it_ImgCtr)
	{

		*ptClToPtrSource += this->ptCls[it_ImgCtr->first].second;		  // cloud_in   // queryImgNbr
		*ptClToPtrTarget += this->ptCls[it_ImgCtr->second.second].second; // testImgNbr

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr GlobalptCl(new pcl::PointCloud<pcl::PointXYZRGB>());

		cout << " Copied to ptrsource and ptClToPtrTarget" << endl;

		// computeTransformation (PointCloudSource &output, const Matrix4 &guess)
		Eigen::MatrixXd icp_cov(6, 6), icp_inf(6, 6);
		icp_cov = Eigen::MatrixXd::Zero(6, 6);
		icp_inf = Eigen::MatrixXd::Zero(6, 6);
		Eigen::Matrix4f iG;
		cbshot::calculate_cov(*ptClToPtrSource, *ptClToPtrTarget, iG, icp_cov);

		this->iGMap[it_ImgCtr->first] = std::make_pair(iG, it_ImgCtr->second.second);
		this->iGMap_cov[it_ImgCtr->first] = std::make_pair(icp_cov, it_ImgCtr->second.second);
		icp_inf = icp_cov.inverse();

		string quat_trans, edges3d;
		ostringstream information_matrix, edges3d_make;
		HelperFcts::PoseToeigenQuatTransl(iG, quat_trans);
		// pose_id x y z q_x q_y q_z q_w
		information_matrix << " " << icp_inf(0, 0) << " " << icp_inf(0, 1) << " " << icp_inf(0, 2) << " " << icp_inf(0, 3)
						   << " " << icp_inf(0, 4) << " " << icp_inf(0, 5) << icp_inf(1, 1) << " " << icp_inf(1, 2) << " "
						   << icp_inf(1, 3) << " " << icp_inf(1, 4) << " " << icp_inf(1, 5) << icp_inf(2, 2) << " "
						   << icp_inf(2, 3) << " " << icp_inf(2, 4) << " " << icp_inf(2, 5) << icp_inf(3, 3) << " "
						   << icp_inf(3, 4) << " " << icp_inf(3, 5) << icp_inf(4, 4) << " " << icp_inf(4, 5)
						   << icp_inf(5, 5);

		edges3d = quat_trans + information_matrix.str(); //" 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1" ;
		this->match_edge[it_ImgCtr->first] = std::make_pair(edges3d, it_ImgCtr->second.second);
		cout << "the edge 3d is" << edges3d << endl;
		cout << endl
			 << "The total number of matching edges are " << this->match_edge.size() << endl;
		if (this->iGMap.size() > 0)
		{
			cbshot::calculate_cov(*P_ptClToPtrSource, *ptClToPtrTarget, iG, icp_cov);
			this->iGMap_croxx[P_SNbr] = std::make_pair(iG, it_ImgCtr->second.second);
			this->iGMap_cov[P_SNbr] = std::make_pair(icp_cov, it_ImgCtr->second.second);
			icp_inf = icp_cov.inverse();

			HelperFcts::PoseToeigenQuatTransl(iG, quat_trans);
			// pose_id x y z q_x q_y q_z q_w
			information_matrix << " " << icp_inf(0, 0) << " " << icp_inf(0, 1) << " " << icp_inf(0, 2) << " " << icp_inf(0, 3)
							   << " " << icp_inf(0, 4) << " " << icp_inf(0, 5) << icp_inf(1, 1) << " " << icp_inf(1, 2) << " "
							   << icp_inf(1, 3) << " " << icp_inf(1, 4) << " " << icp_inf(1, 5) << icp_inf(2, 2) << " "
							   << icp_inf(2, 3) << " " << icp_inf(2, 4) << " " << icp_inf(2, 5) << icp_inf(3, 3) << " "
							   << icp_inf(3, 4) << " " << icp_inf(3, 5) << icp_inf(4, 4) << " " << icp_inf(4, 5)
							   << icp_inf(5, 5);

			edges3d = quat_trans + information_matrix.str(); //" 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1" ;
			this->match_edge[P_SNbr] = std::make_pair(edges3d, it_ImgCtr->second.second);

			cbshot::calculate_cov(*ptClToPtrSource, *P_ptClToPtrTarget, iG, icp_cov);
			this->iGMap_croxx[it_ImgCtr->first] = std::make_pair(iG, P_TNbr);
			this->iGMap_cov[it_ImgCtr->first] = std::make_pair(icp_cov, P_TNbr);
			icp_inf = icp_cov.inverse();

			HelperFcts::PoseToeigenQuatTransl(iG, quat_trans);
			// pose_id x y z q_x q_y q_z q_w
			information_matrix << " " << icp_inf(0, 0) << " " << icp_inf(0, 1) << " " << icp_inf(0, 2) << " " << icp_inf(0, 3)
							   << " " << icp_inf(0, 4) << " " << icp_inf(0, 5) << icp_inf(1, 1) << " " << icp_inf(1, 2) << " "
							   << icp_inf(1, 3) << " " << icp_inf(1, 4) << " " << icp_inf(1, 5) << icp_inf(2, 2) << " "
							   << icp_inf(2, 3) << " " << icp_inf(2, 4) << " " << icp_inf(2, 5) << icp_inf(3, 3) << " "
							   << icp_inf(3, 4) << " " << icp_inf(3, 5) << icp_inf(4, 4) << " " << icp_inf(4, 5)
							   << icp_inf(5, 5);

			edges3d = quat_trans + information_matrix.str(); //" 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1" ;
			this->match_edge[it_ImgCtr->first] = std::make_pair(edges3d, P_TNbr);
		}
		*P_ptClToPtrSource = *ptClToPtrSource;
		*P_ptClToPtrTarget = *ptClToPtrTarget;
		P_SNbr = it_ImgCtr->first;
		P_TNbr = it_ImgCtr->second.second;
	}
}
/**
 * 
 *  Query Robot ID: Query_rID , this->iG_RobotID
 *  P_QPose: Previous Pose of Query Robot
 *  P_TPose: Previous pose of Test Robot
 * 
 * 
 */
void CentralStorage::Apply_Optimization()
{

	cout << "Appling Optimization ..., Please wait" << endl;
	// Scale
	Eigen::Matrix4f scale(Eigen::Matrix4f::Identity(4, 4));
	// robotScales<robot ID, Scale>, means scale of first robot
	scale.block(0, 0, 3, 3) = 1 / this->robotScales[1] * Eigen::Matrix3f::Identity(3, 3);

	// **localPoses** <queryimgnumber, <robotID, pose>

	// Findout the Robot ID
	int Query_rID = this->iG_RobotID;
	cout << "Query robot ID = " << Query_rID << endl;

	string quat_trans;

	Eigen::Matrix4f P_QPose(Eigen::Matrix4f::Identity(4, 4));
	Eigen::Matrix4f P_TPose(Eigen::Matrix4f::Identity(4, 4));
	int P_QPoseNbr = 0;
	int P_TPoseNbr = 0;

	ostringstream matched_infoo;
	matched_infoo << "Query Robot" << setw(15) << "Robot ID" << setw(15) << "Renamed";
	this->Matching_info.push_back(matched_infoo.str());

	int ig_map_size = this->iGMap.size();
	int ig_query = 0;
	int rename_Node = 0;

	//! int to not take the last edge;
	int check_last_edge = 0;

	/**
   *? Iterate the iGmap (which have all direct matches)
   */
	typedef std::map<int, std::pair<Eigen::Matrix4f, int>> iG_Map_poses; // <queryimgnumber, <iG, testImgNbr>
	for (iG_Map_poses::iterator iGMap_iterator = this->iGMap.begin(); iGMap_iterator != this->iGMap.end();
		 ++iGMap_iterator)
	{

		// Query Nodes and Edges
		if (ig_query == 0) //? Very first pose (when there is no previous)
		{
			typedef std::map<int, std::pair<int, geometry_msgs::PoseStamped>> poses_local; // <queryimgnumber, <robotID,
																						   // pose>
			for (poses_local::iterator poses_iterator = this->localPoses.begin(); poses_iterator != this->localPoses.end();
				 ++poses_iterator)
			{
				/**
		 		* 
		 		*? Iterate till first match (iGMap_iterator->first) of the query robot (poses_iterator->second.first == Query_rID)
		 		*/
				if (poses_iterator->first < iGMap_iterator->first && poses_iterator->second.first == Query_rID)
				{

					// Query NODES
					// Node Value
					Eigen::Matrix4f Node_localPose(Eigen::Matrix4f::Identity(4, 4));
					HelperFcts::poseStampedROSToMatrix4f(poses_iterator->second.second, Node_localPose);
					this->rename_Nodes.insert(std::pair<int, int>(poses_iterator->first, rename_Node));
					HelperFcts::localPoseTog2o(poses_iterator->second.second, this->rename_Nodes.at(poses_iterator->first),
											   quat_trans); // rosmsg, queryimg, string

					this->SE3_g2o.push_back(quat_trans); // push the message VERTEX_SE3:QUAT queryImgNbr Quaternion Pose

					/**
					 *? For the Output file who have rename node and robot id for each keyframe
					*/
					ostringstream matched_infoo;
					matched_infoo << poses_iterator->first << setw(20) << poses_iterator->second.first << setw(15) << rename_Node; //queryimg, robotID and rename value for the output file
					this->Matching_info.push_back(matched_infoo.str());

					// cout << "1 Quat_trans" << quat_trans << endl;
					rename_Node = rename_Node + 1;
					cout << "size of SE3_g2o " << this->SE3_g2o.size() << endl;

					// poses_query_image <= match query image   && first value of igMAp or last value of igMAP

					// copy the previous poses of query robot.

					//	cout << "rename_Node " << rename_Node << endl;
					//	this->rename_Nodes.insert(std::pair<int, int>(poses_iterator->first, rename_Node));
					/**
					 *? Calculate the Query Edges between previous to next(current) pose
					*/
					if (this->SE3_g2o.size() >= 2)
					{

						//	Eigen::Matrix4f P_QPose_transpose(Eigen::Matrix4f::Identity(4, 4));
						//	HelperFcts::invTrafo(P_QPose, P_QPose_transpose); //Take the inverse transform
						// TPose_Tranformation = Eigen::Matrix4f::Identity(4, 4) * P_QPose_transpose * Node_localPose * scale;
						//	cout << "P_QPose_transpose" << endl;
						// Calculating the transformation of robot2(test)

						Eigen::Matrix4f TPose_Tranformation(Eigen::Matrix4f::Identity(4, 4));
						HelperFcts::poses2edge(P_QPose, Node_localPose, TPose_Tranformation);

						// !	TPose_Tranformation = TPose_Tranformation * scale;

						HelperFcts::PoseToeigenQuatTransl(TPose_Tranformation, quat_trans);
						ostringstream edges3d_make;
						// Nodes3d_make << "VERTEX_SE3:QUAT "  << this->rename_Nodes.at(iG_iterator->second.second);

						edges3d_make << "EDGE_SE3:QUAT " << this->rename_Nodes.at(P_QPoseNbr) << " "
									 << this->rename_Nodes.at(poses_iterator->first);
						string edges3d = edges3d_make.str();
						edges3d = edges3d + quat_trans;
						edges3d = edges3d + " 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1";
						//	edges3d = edges3d + " 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0" ;
						this->SE3_g2o.push_back(edges3d);
						cout << "835-------- " << edges3d << endl;
					}
					// rename nodes here
					P_QPose = Node_localPose;
					P_QPoseNbr = poses_iterator->first;
				}
			}
		}

		/** IG Query Nodes and Edges
		/
		*? When equal to the iGMaps first value
		*/
		typedef std::map<int, std::pair<int, geometry_msgs::PoseStamped>> poses_local; // <queryimgnumber, <robotID, pose>
		for (poses_local::iterator poses_iterator = this->localPoses.begin(); poses_iterator != this->localPoses.end();
			 ++poses_iterator)
		{

			if (poses_iterator->first == iGMap_iterator->first)
			{
				//cout << ".......IG 856 QUERY Poses......" << endl;
				//	cout << "iGMAP Iterator Query = " << iGMap_iterator->first << "   and Pose Iterator = " <<
				// poses_iterator->first << endl;
				// Query NODES
				//? Get Node's Pose
				Eigen::Matrix4f Node_localPose(Eigen::Matrix4f::Identity(4, 4));
				HelperFcts::poseStampedROSToMatrix4f(poses_iterator->second.second, Node_localPose);

				//	cout << "...... Query Poses ...... " << endl;
				//? Rename pose
				this->rename_Nodes.insert(std::pair<int, int>(poses_iterator->first, rename_Node));
				HelperFcts::localPoseTog2o(poses_iterator->second.second, this->rename_Nodes.at(poses_iterator->first), quat_trans); // Pose, renamed-queryimg, string(output)
				this->SE3_g2o.push_back(quat_trans);

				//? For the renamed file
				ostringstream matched_infoo;
				matched_infoo << poses_iterator->first << setw(20) << poses_iterator->second.first << setw(15) << rename_Node;
				this->Matching_info.push_back(matched_infoo.str());
				rename_Node = rename_Node + 1;

				//cout << "876 size of SE3_g2o " << this->SE3_g2o.size() << endl;

				// Query Edges

				// poses_query_image <= match query image   && first value of igMAp or last value of igMAP

				// copy the previous poses of query robot.

				//	cout << "rename_Node " << rename_Node << endl;
				//	this->rename_Nodes.insert(std::pair<int, int>(poses_iterator->first, rename_Node));

				//	Eigen::Matrix4f P_QPose_transpose(Eigen::Matrix4f::Identity(4, 4));
				// HelperFcts::invTrafo(P_QPose, P_QPose_transpose); //Take the inverse transform
				//	cout << "P_QPose_transpose" << endl;
				// Calculating the transformation of robot2(test)

				/**
				 *? Edges between IGMap Query poses 
				*/
				Eigen::Matrix4f TPose_Tranformation(Eigen::Matrix4f::Identity(4, 4));
				//	TPose_Tranformation = Eigen::Matrix4f::Identity(4, 4) * P_QPose_transpose * Node_localPose * scale; // ;
				HelperFcts::poses2edge(P_QPose, Node_localPose, TPose_Tranformation);
				//TPose_Tranformation = TPose_Tranformation * scale;

				//cout << "900 QPose_Tranformation" << endl;

				HelperFcts::PoseToeigenQuatTransl(TPose_Tranformation, quat_trans);
				ostringstream edges3d_make;

				//if (!check_last_edge)
				//{
					edges3d_make << "EDGE_SE3:QUAT " << this->rename_Nodes.at(P_QPoseNbr) << " " << this->rename_Nodes.at(poses_iterator->first);
					string edges3d = edges3d_make.str();
					edges3d = edges3d + quat_trans;
					edges3d = edges3d + " 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1";
					// edges3d = edges3d + " 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0" ;

					this->SE3_g2o.push_back(edges3d);

					cout << "917-------- " << edges3d << " " << check_last_edge << endl;
				//}

				// rename nodes here

				P_QPose = Node_localPose;
				P_QPoseNbr = poses_iterator->first;
			}
			check_last_edge++;
		}

		/** / IG Test Nodes and Edges
	/
	*? When equal to the iGMaps 2nd value
	*/
		typedef std::map<int, std::pair<int, geometry_msgs::PoseStamped>> poses_local; // <queryimgnumber, <robotID, pose>
		for (poses_local::iterator poses_iterator = this->localPoses.begin(); poses_iterator != this->localPoses.end();
			 ++poses_iterator)
		{

			if (poses_iterator->first == iGMap_iterator->second.second)
			{

				//	cout << ".......IG Test Poses......" << endl;

				// Calculating the transformation of robot2(test)
				Eigen::Matrix4f TPose_Tranformation(Eigen::Matrix4f::Identity(4, 4));
				//	TPose_Tranformation = Eigen::Matrix4f::Identity(4, 4) * P_QPose * iGMap_iterator->second.first * scale; //
				// P_exprected = iG * P_query
				HelperFcts::pose2pose(P_QPose, iGMap_iterator->second.first, TPose_Tranformation);

				//! Not using the scale
				//TPose_Tranformation = TPose_Tranformation * scale;

				HelperFcts::PoseToeigenQuatTransl(TPose_Tranformation, quat_trans);
				ostringstream Nodes3d_make;

				this->rename_Nodes.insert(std::pair<int, int>(iGMap_iterator->second.second, rename_Node));
				// Nodes3d_make << "VERTEX_SE3:QUAT "  << this->rename_Nodes.at(iG_iterator->second.second);
				//	cout << "iGMap_iterator->second.second " << iGMap_iterator->second.second;
				//	cout << " rename nodes value " << this->rename_Nodes.at(iGMap_iterator->second.second) << endl;
				//	cout << "rename Counter " << rename_Node << endl;
				Nodes3d_make << "VERTEX_SE3:QUAT " << this->rename_Nodes.at(iGMap_iterator->second.second);
				string Nodes3d = Nodes3d_make.str();
				Nodes3d = Nodes3d + quat_trans;
				cout << ":::::Testtttt Nodes " << Nodes3d << endl;
				this->SE3_g2o.push_back(Nodes3d);

				//? For the matching file
				ostringstream matched_infoo;
				matched_infoo << poses_iterator->first << setw(20) << poses_iterator->second.first << setw(15) << rename_Node;
				this->Matching_info.push_back(matched_infoo.str());

				rename_Node = rename_Node + 1;

				/**
		 *? Edges between IGMap test poses 
		 */
				Eigen::Matrix4f TNode_localPose(Eigen::Matrix4f::Identity(4, 4));
				HelperFcts::poseStampedROSToMatrix4f(poses_iterator->second.second, TNode_localPose);

				if (P_TPoseNbr != 0)
				{
					// Eigen::Matrix4f P_TPose_transpose(Eigen::Matrix4f::Identity(4, 4));
					// HelperFcts::invTrafo(P_TPose, P_TPose_transpose); //Take the inverse transform
					//		cout << "Test 2nd Node entered... " << endl;
					// Calculating the transformation of robot2(test)
					Eigen::Matrix4f TPose_Tranformation(Eigen::Matrix4f::Identity(4, 4));
					// TPose_Tranformation = Eigen::Matrix4f::Identity(4, 4) * P_TPose_transpose * TNode_localPose * scale; // ;

					HelperFcts::poses2edge(P_TPose, TNode_localPose, TPose_Tranformation);
					//TPose_Tranformation = TPose_Tranformation * scale;

					HelperFcts::PoseToeigenQuatTransl(TPose_Tranformation, quat_trans);
					ostringstream edges3d_make;

					edges3d_make << "EDGE_SE3:QUAT " << this->rename_Nodes.at(P_TPoseNbr) << " " << this->rename_Nodes.at(iGMap_iterator->second.second);
					string edges3d = edges3d_make.str();
					edges3d = edges3d + quat_trans;
					edges3d = edges3d + " 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1";
					// edges3d = edges3d + " 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0" ;
					this->SE3_g2o.push_back(edges3d);
					//  cout << edges3d << endl;
					cout << "998-------- " << edges3d << " " << check_last_edge << endl;
				}

				P_TPose = TNode_localPose;
				P_TPoseNbr = iGMap_iterator->second.second;
			}
		}

		cout << "ig Query " << ig_query << " and ig map size - 1 = " << ig_map_size - 1 << endl;
		// Query Nodes and Edges
		//? Last iGMap matching
		if (ig_query == ig_map_size - 1)
		{

			typedef std::map<int, std::pair<int, geometry_msgs::PoseStamped>> poses_local; // <queryimgnumber, <robotID,
																						   // pose>
			for (poses_local::iterator poses_iterator = this->localPoses.begin(); poses_iterator != this->localPoses.end();
				 ++poses_iterator)
			{

				if (poses_iterator->first > iGMap_iterator->first && poses_iterator->second.first == Query_rID)
				{

					//	cout << "iGMAP Iterator Query = " << iGMap_iterator->first << "   and Pose Iterator = " <<
					// poses_iterator->first << endl;
					// Query NODES
					// Node Value
					Eigen::Matrix4f Node_localPose(Eigen::Matrix4f::Identity(4, 4));
					HelperFcts::poseStampedROSToMatrix4f(poses_iterator->second.second, Node_localPose);

					cout << "...... LAST ...... " << endl;
					this->rename_Nodes.insert(std::pair<int, int>(poses_iterator->first, rename_Node));
					HelperFcts::localPoseTog2o(poses_iterator->second.second, this->rename_Nodes.at(poses_iterator->first),
											   quat_trans); // rosmsg, queryimg, string
					this->SE3_g2o.push_back(quat_trans);
					ostringstream matched_infoo;
					matched_infoo << poses_iterator->first << setw(20) << poses_iterator->second.first << setw(15) << rename_Node;
					this->Matching_info.push_back(matched_infoo.str());
					cout << "1 Quat_trans" << quat_trans << endl;
					rename_Node = rename_Node + 1;

					if (this->SE3_g2o.size() >= 2)
					{

						//	Eigen::Matrix4f P_QPose_transpose(Eigen::Matrix4f::Identity(4, 4));
						//	HelperFcts::invTrafo(P_QPose, P_QPose_transpose); //Take the inverse transform
						//	cout << "P_QPose_transpose" << endl;
						// Calculating the transformation of robot2(test)
						Eigen::Matrix4f TPose_Tranformation(Eigen::Matrix4f::Identity(4, 4));
						// TPose_Tranformation = Eigen::Matrix4f::Identity(4, 4) * P_QPose_transpose * Node_localPose * scale; // ;

						HelperFcts::poses2edge(P_QPose, Node_localPose, TPose_Tranformation);
						//! TPose_Tranformation = TPose_Tranformation * scale;

						// cout << "QPose_Tranformation" << endl;

						HelperFcts::PoseToeigenQuatTransl(TPose_Tranformation, quat_trans);
						ostringstream edges3d_make;
						// Nodes3d_make << "VERTEX_SE3:QUAT "  << this->rename_Nodes.at(iG_iterator->second.second);

						edges3d_make << "EDGE_SE3:QUAT " << this->rename_Nodes.at(P_QPoseNbr) << " "
									 << this->rename_Nodes.at(poses_iterator->first);
						string edges3d = edges3d_make.str();
						edges3d = edges3d + quat_trans;
						edges3d = edges3d + " 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1";
						//edges3d = edges3d + " 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0";
						this->SE3_g2o.push_back(edges3d);
						cout << "Last " << edges3d << endl;

						//	rename_Node = rename_Node + 1;
					}
					// rename nodes here
					P_QPose = Node_localPose;
					P_QPoseNbr = poses_iterator->first;
				}
			}
		}

		ig_query = ig_query + 1;
	}

	cout << "END END END" << endl;

	//? Matches Egdes - Rename the direct edges and push to SE3_g2o for optimization

	typedef std::map<int, std::pair<string, int>> iG_Map_edges; // <queryimgnumber, <edge, testImgNbr>
	for (iG_Map_edges::iterator iGEdges_iterator = this->match_edge.begin(); iGEdges_iterator != this->match_edge.end();
		 ++iGEdges_iterator)
	{

		ostringstream edges3d_make;
		//cout << "iGEdges_iterator FIRST " << iGEdges_iterator->first << "TEST :" << iGEdges_iterator->second.second << endl;

		edges3d_make << "EDGE_SE3:QUAT " << this->rename_Nodes.at(iGEdges_iterator->first) << " "
					 << this->rename_Nodes.at(iGEdges_iterator->second.second);
		string edges3d = edges3d_make.str();
		edges3d = edges3d + iGEdges_iterator->second.first;
		this->SE3_g2o.push_back(edges3d);
		cout << "edge3d: " << edges3d << endl;
	}

	//? Top left to down right
	for (iG_Map_edges::iterator iGEdges_iterator_ldr = this->match_edge_leftdownright.begin(); iGEdges_iterator_ldr != this->match_edge_leftdownright.end();
		 ++iGEdges_iterator_ldr)
	{

		ostringstream edges3d_make;
		//cout << "iGEdges_iterator_ldr FIRST " << iGEdges_iterator_ldr->first << "TEST :" << iGEdges_iterator_ldr->second.second << endl;

		edges3d_make << "EDGE_SE3:QUAT " << this->rename_Nodes.at(iGEdges_iterator_ldr->first) << " "
					 << this->rename_Nodes.at(iGEdges_iterator_ldr->second.second);
		string edges3d = edges3d_make.str();
		edges3d = edges3d + iGEdges_iterator_ldr->second.first;
		this->SE3_g2o.push_back(edges3d);
		cout << "edge3d: " << edges3d << endl;
	}

	//? Top right to down left
	for (iG_Map_edges::iterator iGEdges_iterator_rdl = this->match_edge_rightdownleft.begin(); iGEdges_iterator_rdl != this->match_edge_rightdownleft.end();
		 ++iGEdges_iterator_rdl)
	{

		ostringstream edges3d_make;
		//cout << "iGEdges_iterator_rdl FIRST " << iGEdges_iterator_rdl->first << "TEST :" << iGEdges_iterator_rdl->second.second << endl;

		edges3d_make << "EDGE_SE3:QUAT " << this->rename_Nodes.at(iGEdges_iterator_rdl->first) << " "
					 << this->rename_Nodes.at(iGEdges_iterator_rdl->second.second);
		string edges3d = edges3d_make.str();
		edges3d = edges3d + iGEdges_iterator_rdl->second.first;
		this->SE3_g2o.push_back(edges3d);
		cout << "edge3d: " << edges3d << endl;
	}

	HelperFcts::Makeg2oFile(this->SE3_g2o, "matchingImgs/Optimization.g2o");
	HelperFcts::Makeg2oFile(this->Matching_info, "matchingImgs/Matching_b4_optimization.txt");

	cout << "Pose graph optimization started..." << endl;
	ceres::examples::MapOfPoses poses;
	ceres::examples::VectorOfConstraints constraints;
	ceres::examples::ReadG2oFile("matchingImgs/Optimization.g2o", &poses, &constraints);
	cout << "Number of poses: " << poses.size() << '\n';

	cout << "Number of constraints: " << constraints.size() << '\n';

	ceres::examples::OutputPoses("matchingImgs/poses_original.txt", poses);
	ceres::examples::Final_Pose(poses, this->OriginalPoses);
	ceres::Problem problem;
	ceres::examples::BuildOptimizationProblem(constraints, &poses, &problem);
	ceres::examples::SolveOptimizationProblem(&problem);
	ceres::examples::OutputPoses("matchingImgs/poses_optimized.txt", poses);

	// output is the "this->OptimizedPosesinWorld"
	ceres::examples::Final_Pose(poses, this->OptimizedPosesinWorld);

	// cout << "the size of Optimized Poses in World " << this->OptimizedPosesinWorld.size() << endl;
	int NodeName;
	// Findout the Robot ID

	typedef std::map<int, Eigen::Matrix4f> poses_map; // std::map<int, Eigen::Matrix4f> OptimizedPosesinWorld
	for (poses_map::iterator poses_iterator = this->OptimizedPosesinWorld.begin();
		 poses_iterator != this->OptimizedPosesinWorld.end(); ++poses_iterator)
	{

		// to replace with the new poses
		for (auto &x : rename_Nodes)
		{
			if (x.second == poses_iterator->first) //
				NodeName = x.first;
		}

		keyFrame iG_Query_KF = this->kFrames[NodeName];
		int Query_robotID = iG_Query_KF.rID;
		HelperFcts::print4x4Matrix((poses_iterator->second).cast<double>());

		this->localOptimizedPoses[NodeName] = std::make_pair(Query_robotID, poses_iterator->second);
		cout << "NodeName inserted" << NodeName << " of robot " << Query_robotID << endl;
		cout << "Size of local optimized poses " << this->localOptimizedPoses.size() << endl;
	}

	// Find the Roots of the Test Robot (apply Optimization to the roots)

	int Test_iG_First = this->iGMap.begin()->second.second;

	// Reverse Loop

	typedef std::map<int, std::pair<int, geometry_msgs::PoseStamped>> rposes_map;
	for (rposes_map::reverse_iterator poses_iterator = this->localPoses.rbegin();
		 poses_iterator != this->localPoses.rend(); ++poses_iterator)
	{
		if (poses_iterator->second.first != Query_rID)
		{
			Eigen::Matrix4f New_localPose(Eigen::Matrix4f::Identity(4, 4));
			HelperFcts::poseStampedROSToMatrix4f(poses_iterator->second.second, New_localPose);
			if (poses_iterator->first < Test_iG_First)
			{

				Eigen::Matrix4f PoseEdge(Eigen::Matrix4f::Identity(4, 4));
				HelperFcts::poses2edge(P_TPose, New_localPose, PoseEdge);

				// now apply the pose edge to the optimized pose
				Eigen::Matrix4f CalculatedPose(Eigen::Matrix4f::Identity(4, 4));
				HelperFcts::pose2pose(this->localOptimizedPoses[Test_iG_First].second, PoseEdge,
									  CalculatedPose); // P2 = P1*Edge;

				HelperFcts::print4x4Matrix((CalculatedPose).cast<double>());
				//! comment due to not taking the previous poses of test robots
				// this->localOptimizedPoses[poses_iterator->first] = std::make_pair(poses_iterator->second.first,
				// CalculatedPose );
				cout << "NodeName inserted: " << poses_iterator->first << " of robot " << poses_iterator->second.first << endl;
				cout << "Size of local optimized poses: " << this->localOptimizedPoses.size() << endl;
				ostringstream matched_infoo;
				matched_infoo << Test_iG_First << " " << poses_iterator->first;

				this->Matching_info.push_back(matched_infoo.str());
				Test_iG_First = poses_iterator->first;
			}
			P_TPose = New_localPose;
		}
	}

	HelperFcts::Makeg2oFile(this->Matching_info, "matchingImgs/Matching_info.txt");
}

// ICP apply to points cloud directly

void CentralStorage::PointCloud_Fusion()
{

	cout << "Point Cloud Fusion Started..." << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptClToPtrSource(
		new pcl::PointCloud<pcl::PointXYZ>()); // Original point cloud (cloud_icp = ptClToPtrSource )
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptClToPtrTarget(
		new pcl::PointCloud<pcl::PointXYZ>()); // ICP output point cloud (cloud_in = ptClToPtrTarget )

	pcl::PointCloud<pcl::PointXYZ>::Ptr GlobalptCl_Clean(
		new pcl::PointCloud<pcl::PointXYZ>()); // ICP output point cloud (cloud_in = ptClToPtrTarget )

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GlobalptCl(new pcl::PointCloud<pcl::PointXYZRGB>());

	int NodeName;

	typedef std::map<int, std::pair<int, Eigen::Matrix4f>> poses_map; // std::map<int, Eigen::Matrix4f>
																	  // OptimizedPosesinWorld
	// localOptimizedPoses: <Nodes , <robot ID, OptimizedPose>
	for (poses_map::iterator poses_iterator = this->localOptimizedPoses.begin();
		 poses_iterator != this->localOptimizedPoses.end(); ++poses_iterator)
	{

		NodeName = poses_iterator->first;
		Eigen::Matrix4f original_local_pose(Eigen::Matrix4f::Identity(4, 4));
		HelperFcts::poseStampedROSToMatrix4f(this->localPoses[NodeName].second, original_local_pose);

		Eigen::Matrix4f PoseEdge(Eigen::Matrix4f::Identity(4, 4));
		HelperFcts::poses2edge(original_local_pose, poses_iterator->second.second, PoseEdge);

		pcl::transformPointCloud(this->ptCls[NodeName].second, this->GlobalWorldPtCls[NodeName].second, PoseEdge);

		//	HelperFcts::print4x4Matrix((PoseEdge).cast<double>());

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PtCLRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::copyPointCloud(this->GlobalWorldPtCls[NodeName].second, *PtCLRGB);

		*GlobalptCl_Clean += this->GlobalWorldPtCls[NodeName].second;
		// ptCls:   ImgCtr <robot ID,pcl>

		if (poses_iterator->second.first == 1)
		{

			// Source Point cloud
			*ptClToPtrSource += this->GlobalWorldPtCls[NodeName].second; // cloud_in   // queryImgNbr

			//	uint8_t r = 255, g = 0, b = 0;    // Example: Red color
			for (size_t i = 0; i < PtCLRGB->points.size(); ++i)
			{
				PtCLRGB->points[i].r = 255;
				PtCLRGB->points[i].g = 0;
				PtCLRGB->points[i].b = 0;
			}
		}
		else if (poses_iterator->second.first == 2)
		{

			// Tagget point cloud

			*ptClToPtrTarget += this->GlobalWorldPtCls[NodeName].second; // cloud_in   // queryImgNbr
			for (size_t i = 0; i < PtCLRGB->points.size(); ++i)
			{
				PtCLRGB->points[i].r = 0;
				PtCLRGB->points[i].g = 255;
				PtCLRGB->points[i].b = 0;
			}
		}

		*GlobalptCl += *PtCLRGB; // cloud_in   // queryImgNbr
	}

	pcl::io::savePLYFileASCII("matchingImgs/ptClToPtrSource.ply", *ptClToPtrSource);
	pcl::io::savePLYFileASCII("matchingImgs/ptClToPtrTarget.ply", *ptClToPtrTarget);
	pcl::io::savePLYFileASCII("matchingImgs/final.ply", *GlobalptCl_Clean);
	pcl::io::savePLYFileASCII("matchingImgs/finalrgb.ply", *GlobalptCl);
}
/*void CentralStorage::PointCloud_Fusion() {

	cout << "Point Cloud Fusion Started..." << endl;


	pcl::PointCloud<pcl::PointXYZ>::Ptr ptClToPtrSource(new pcl::PointCloud<pcl::PointXYZ>()); // Original point cloud
(cloud_icp = ptClToPtrSource )
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptClToPtrTarget(new pcl::PointCloud<pcl::PointXYZ>()); // ICP output point cloud
(cloud_in = ptClToPtrTarget )

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GlobalptCl(new pcl::PointCloud<pcl::PointXYZRGB>());

	int NodeName;

	// Find out the edges from orignal to optimzed , to be used in the transfomration of the point cloud
	typedef std::map<int, Eigen::Matrix4f> original_poses_map;
	for (original_poses_map::iterator original_poses_iterator = this->OriginalPoses.begin(); original_poses_iterator !=
this->OriginalPoses.end(); ++original_poses_iterator) {
		typedef std::map<int, Eigen::Matrix4f> poses_map; // std::map<int, Eigen::Matrix4f> OptimizedPosesinWorld
		for (poses_map::iterator poses_iterator = this->OptimizedPosesinWorld.begin(); poses_iterator !=
this->OptimizedPosesinWorld.end(); ++poses_iterator) {


			if (original_poses_iterator->first == poses_iterator->first) {
				// to find out the original names
				for (auto& x : rename_Nodes) {
					if (x.second == original_poses_iterator->first)
						NodeName =  x.first;

				}
				cout << "NOdeNAME " << NodeName << " (it-> First)" << poses_iterator->first << endl;
				Eigen::Matrix4f original_pose_transpose(Eigen::Matrix4f::Identity(4, 4));
				HelperFcts::invTrafo(original_poses_iterator->second, original_pose_transpose);

				Eigen::Matrix4f Optimized_Pose(Eigen::Matrix4f::Identity(4, 4));
				Optimized_Pose = poses_iterator->second;


				Eigen::Matrix4f Trans_Edge;
				Trans_Edge = Eigen::Matrix4f::Identity(4, 4) * Optimized_Pose * original_pose_transpose;

				// Save the value with the original names
				this->OptimizedPoses_Edges[NodeName] = Trans_Edge;
				//	cout << "Size of the Optimized poses edges = " << this->OptimizedPoses_Edges.size() << endl;

				pcl::transformPointCloud(this->ptCls[NodeName].second, this->GlobalWorldPtCls[NodeName].second,
Trans_Edge);
				this->GlobalWorldPtCls[NodeName].first = this->ptCls[NodeName].first;
				cout << "Robot ID " << this->GlobalWorldPtCls[NodeName].first << endl;


			}

		}
	}



	int Query_rID = this->iG_RobotID;
	cout << "iG_RobotID" << this->iG_RobotID << endl;
	int P_test_nbr = 0;
	int ig_Query = 0;
	// IGMAP<queryimgnumber, <iG, testImgNbr>
	typedef std::map<int, std::pair <Eigen::Matrix4f, int> > iG_Map_poses;
	for (iG_Map_poses::iterator iGMap_iterator = this->iGMap.begin(); iGMap_iterator != this->iGMap.end();
++iGMap_iterator) {

		// ptCls:   ImgCtr <robot ID,pcl>
		typedef std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > > ptClMap;
		for (ptClMap::iterator it = this->ptCls.begin(); it != this->ptCls.end(); ++it) {

			// Check for the Test Point Clouds Only;
			if (it->second.first != Query_rID) {

				if (ig_Query == 0 && it->first < iGMap_iterator->second.second) {
					//Previous Copy

					pcl::transformPointCloud(it->second.second, this->GlobalWorldPtCls[it->first].second,
this->OptimizedPoses_Edges[iGMap_iterator->second.second]);
					this->GlobalWorldPtCls[it->first].first = this->ptCls[it->first].first;
					cout << "ig_Query == 0 && it->first < iGMap_iterator->second.second = " << it->first << endl;

				} else {
					if ((it->first > P_test_nbr) && (it->first < iGMap_iterator->second.second)) {
						// Copy in between
						pcl::transformPointCloud(it->second.second, this->GlobalWorldPtCls[it->first].second,
this->OptimizedPoses_Edges[iGMap_iterator->second.second]);
						this->GlobalWorldPtCls[it->first].first = this->ptCls[it->first].first;
						cout << "it->first > P_test_nbr && it->first < iGMap_iterator->second.second = " << it->first <<
endl;

					}
					if ((it->first > P_test_nbr) && (ig_Query == this->iGMap.size() - 1) && (it->first >
iGMap_iterator->second.second)) {
						//Copy the LAst one
						pcl::transformPointCloud(it->second.second, this->GlobalWorldPtCls[it->first].second,
this->OptimizedPoses_Edges[iGMap_iterator->second.second]);
						this->GlobalWorldPtCls[it->first].first = this->ptCls[it->first].first;
						cout << "it->first > P_test_nbr && ig_Query == this->iGMap.size() - 1 = " << it->first << endl;
					}
				}

			}
		}
		P_test_nbr = iGMap_iterator->second.second;
		ig_Query = ig_Query + 1;
		cout << "ig_Query " << ig_Query << endl;
		cout << "iGMap_iterator " << iGMap_iterator->second.second << endl;
	}


	// Saving the Colors
	typedef std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > > ptClMap;
	for (ptClMap::iterator it = this->GlobalWorldPtCls.begin(); it != this->GlobalWorldPtCls.end(); ++it) {



		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PtCLRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::copyPointCloud(it->second.second, *PtCLRGB);

		// ptCls:   ImgCtr <robot ID,pcl>
		if (it->second.first == Query_rID)
		{

			// Source Point cloud
			*ptClToPtrSource += it->second.second; //cloud_in   // queryImgNbr
			cout << "Query  " << it->first << "Robot ID = " << it->second.first << endl;

			//  uint8_t r = 255, g = 0, b = 0;    // Example: Red color
			for (size_t i = 0; i < PtCLRGB->points.size (); ++i) {
				PtCLRGB->points[i].r = 255;
				PtCLRGB->points[i].g = 0;
				PtCLRGB->points[i].b = 0;
			}


		} else if (it->second.first != Query_rID) {

			// Tagget point cloud

			*ptClToPtrTarget += it->second.second; //cloud_in   // queryImgNbr
			cout << "Test  " << it->first << "Robot ID = " << it->second.first << endl;
			//green color
			for (size_t i = 0; i < PtCLRGB->points.size (); ++i) {
				PtCLRGB->points[i].r = 0;
				PtCLRGB->points[i].g = 255;
				PtCLRGB->points[i].b = 0;
			}


		}


		*GlobalptCl += *PtCLRGB; //cloud_in   // queryImgNbr

	}

	pcl::io::savePLYFileASCII ("matchingImgs/ptClToPtrSource.ply", *ptClToPtrSource);
	pcl::io::savePLYFileASCII ("matchingImgs/ptClToPtrTarget.ply", *ptClToPtrTarget);
	pcl::io::savePLYFileASCII ("matchingImgs/final.ply", *GlobalptCl);


}*/

void CentralStorage::updatePosesOfRobots()
{
	cout << "4: + New Poses Found" << endl;

	/*	typedef std::map<int, Eigen::Matrix4f> original_poses_map;
	  for (original_poses_map::iterator original_poses_iterator = this->localPoses.begin(); original_poses_iterator !=
	 this->localPoses.end(); ++original_poses_iterator) {
		  typedef std::map<int, Eigen::Matrix4f> poses_map; // std::map<int, Eigen::Matrix4f> OptimizedPosesinWorld
		  for (poses_map::iterator poses_iterator = this->OptimizedPosesinWorld.begin(); poses_iterator !=
	 this->OptimizedPosesinWorld.end(); ++poses_iterator) {


			  if (original_poses_iterator->first == poses_iterator->first) {
				  // to find out the original names
				  for (auto& x : rename_Nodes) {
					  if (x.second == original_poses_iterator->first)
						  NodeName =  x.first;
				  }

				  Eigen::Matrix4f original_pose_transpose(Eigen::Matrix4f::Identity(4, 4));
				  HelperFcts::invTrafo(original_poses_iterator->second, original_pose_transpose);

				  Eigen::Matrix4f Optimized_Pose(Eigen::Matrix4f::Identity(4, 4));
				  Optimized_Pose = poses_iterator->second;


				  Eigen::Matrix4f Trans_Edge;
				  Trans_Edge = Eigen::Matrix4f::Identity(4, 4) * Optimized_Pose * original_pose_transpose;

				  // Save the value with the original names
				  this->OptimizedPoses_Edges[NodeName] = Trans_Edge;
				  cout << "Size of the Optimized poses edges = " << this->OptimizedPoses_Edges.size() << endl;

				  pcl::transformPointCloud(this->ptCls[NodeName].second, this->GlobalWorldPtCls[NodeName].second,
	 Trans_Edge);



				  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PtCLRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
				  pcl::copyPointCloud(this->GlobalWorldPtCls[original_poses_iterator->first].second, *PtCLRGB);



		  }
	  }
  */

	/*







	  // check if any ptcl has update neccessary
	  // collect all rID which need to be updated (has false bool)
	  typedef std::map<int, bool> ptClUpdatedMap;
	  vector<int> rIDToUpdate;

	  // ptClsInGlobalWorldCOSUpdated; <robot ID, bool: true if ptcls coming from robot with robot ID rID are updated)
	  for (ptClUpdatedMap::iterator it = this->ptClsInGlobalWorldCOSUpdated.begin(); it !=
	 this->ptClsInGlobalWorldCOSUpdated.end(); ++it) {
		  if (it->second == false) {
			  rIDToUpdate.push_back(it->first);

		  }
	  }
	  //matchedKeyFramesMap.insert(std::pair<int, std::pair <int, int> >(this->testImgCtr, this->matchedKeyFrames));
	  //
	  //
	  //IF not match then POSES of both robots => Identity (4x4)
	  //
	  //
	  //
	  if (this->matchedKeyFramesMap.empty()) { // no matches yet
		  //cout << " No matches yet " << endl;
		  this->pRobotsWorld[1] = Eigen::Matrix4f::Identity(4, 4);

		  Eigen::Matrix4f initTrafo(Eigen::Matrix4f::Identity(4, 4));
		  initTrafo.block(0, 3, 3, 1) << 0, 0, 0;
		  this->pRobotsWorld[2] = initTrafo;
	  }
	  // if there is a MATCH
	  else if ( !this->matchedKeyFramesMap.empty() && !rIDToUpdate.empty() ) { // there is a new match
		  for (int i = 1; i <= this->nbrRobots; ++i) { // iterate through all robots
			  if (i == 1) { // update global pose of robot 1
				  this->pRobotsWorld[i] = Eigen::Matrix4f::Identity(4, 4);
				  this->ptClsInGlobalWorldCOSUpdated[i] = false;
			  }
			  else { // update global pose of robot 2
				  // pRobotsWorld[2] = pRobot2World = tWorldRobot2
				  // = tWorldRobot1 * tRobot1Cam1 * tCam1Cam2 * tCam2Robot2
				  // = Identity     * pCam1Robot1 * pCam2Cam1 * pRobot2Cam2
				  // = Identity     * pCam1Robot1 * pCam2Cam1 * pCam2Robot2^(-1)
				  // Latest Keyframe and queries numbers
				  int matchQueryImgCtr = this->matchedKeyFramesMap.rbegin()->second.first;   //matched keyframe query
	 number
				  int matchTestImgCtr = this->matchedKeyFramesMap.rbegin()->second.second;   //matched keyframe query
	 number of 2nd robots


				  // **localPoses** <queryimgnumber, <robotID, pose>

				  if (this->localPoses[matchQueryImgCtr].first == 1) {
					  // R1 = Query, R2 = Test
					  Eigen::Matrix4f pCam1Robot1(Eigen::Matrix4f::Identity(4, 4));
					  cout << "4-1: ... Performing Pose Transformation of Robot ... " << "Robot ID here" << endl;
					  //CONVERT Quaternion to RT matrix
					  HelperFcts::poseStampedROSToMatrix4f(this->localPoses[matchQueryImgCtr].second, pCam1Robot1);
	 //pose of robot1 in frame of 1

					  // copy local pose of robot from localPoses using the QueryImageCtr to pCam1Robot1

					  Eigen::Matrix4f pCam2Cam1(Eigen::Matrix4f::Identity(4, 4));

					  // Get the transformation
					  pCam2Cam1 = this->Final_iG; //getting the poses (ratation and translation) // transformation from
	 cam2 to cam1

					  Eigen::Matrix4f pRobot2Cam2(Eigen::Matrix4f::Identity(4, 4));
					  Eigen::Matrix4f pCam2Robot2(Eigen::Matrix4f::Identity(4, 4));
					  HelperFcts::poseStampedROSToMatrix4f(this->localPoses[matchTestImgCtr].second, pCam2Robot2);

					  // copy local pose of robot from localPoses using the TestImageCtr to pCam2Robot2


					  HelperFcts::invTrafo(pCam2Robot2, pRobot2Cam2); //Take the inverse transform
					  //this->pRobotsWorld[rID] = Eigen::Matrix4f::Identity(4,4) * pCam1Robot1 * pCam2Cam1 *
	 pRobot2Cam2;

					  Eigen::Matrix4f scale(Eigen::Matrix4f::Identity(4, 4));
					  // robotScales<robot ID, Scale>, means scale of first robot
					  scale.block(0, 0, 3, 3) = 1 / this->robotScales[i] * Eigen::Matrix3f::Identity(3, 3);


					  Eigen::Matrix4f invTmp;
					  HelperFcts::invTrafo(Eigen::Matrix4f::Identity(4, 4) * pCam1Robot1 * pRobot2Cam2 * scale, invTmp);
	 // R1 in coordinate of R2
					  // Not USED

					  this->pRobotsWorld[i] = Eigen::Matrix4f::Identity(4, 4) * pCam1Robot1 * pRobot2Cam2 * scale;
					  //this->pRobotsWorld[i] = Eigen::Matrix4f::Identity(4,4) * pCam1Robot1 * pRobot2Cam2 * 1;
					  //	cout << "this->pRobotsWorld[i]: " << endl << this->pRobotsWorld[i] << endl;
					  //	the map contains the transformation of the associated robot relative to the world COS
					  /// NOTE: The world COS is identical with the COS of the robot with rID 1!!!!!!!!!!
					  //	std::map<int,Eigen::Matrix4f> pRobotsWorld;


				  }
				  else if (this->localPoses[matchQueryImgCtr].first == 2) {
					  // R2 = Query, R1 = Test
					  //
					  cout << "4-2: ==========> Now updateing the poses of Robot2 " << endl;
					  Eigen::Matrix4f pCam1Robot1(Eigen::Matrix4f::Identity(4, 4));
					  HelperFcts::poseStampedROSToMatrix4f(this->localPoses[matchTestImgCtr].second, pCam1Robot1);

					  Eigen::Matrix4f pCam2Cam1(Eigen::Matrix4f::Identity(4, 4));
					  Eigen::Matrix4f pCam1Cam2(Eigen::Matrix4f::Identity(4, 4));
					  pCam1Cam2 = this->Final_iG;
					  HelperFcts::invTrafo(pCam1Cam2, pCam2Cam1);

					  Eigen::Matrix4f pRobot2Cam2(Eigen::Matrix4f::Identity(4, 4));
					  Eigen::Matrix4f pCam2Robot2(Eigen::Matrix4f::Identity(4, 4));
					  HelperFcts::poseStampedROSToMatrix4f(this->localPoses[matchQueryImgCtr].second, pCam2Robot2);
					  HelperFcts::invTrafo(pCam2Robot2, pRobot2Cam2);
					  //this->pRobotsWorld[rID] = Eigen::Matrix4f::Identity(4,4) * pCam1Robot1 * pCam2Cam1 *
	 pRobot2Cam2;

					  Eigen::Matrix4f scale(Eigen::Matrix4f::Identity(4, 4));
					  scale.block(0, 0, 3, 3) = 1 / this->robotScales[i] * Eigen::Matrix3f::Identity(3, 3);

					  Eigen::Matrix4f invTmp;
					  HelperFcts::invTrafo(Eigen::Matrix4f::Identity(4, 4) * pCam1Robot1 * pRobot2Cam2 * scale, invTmp);

					  this->pRobotsWorld[i] = Eigen::Matrix4f::Identity(4, 4) * pCam1Robot1 * pRobot2Cam2 * scale;
					  //this->pRobotsWorld[i] = Eigen::Matrix4f::Identity(4,4) * pCam1Robot1 * pRobot2Cam2 * 1;
					  //cout << "this->pRobotsWorld[i]: " << endl <<  this->pRobotsWorld[i] << endl;

				  }
				  this->ptClsInGlobalWorldCOSUpdated[i] = false;
			  }
		  }
	  }*/
}

void CentralStorage::updatePtCls()
{
	cout << "5: +++ New point Cloud Found ..." << endl;

	/* Naming conventions:
   * p: pose
   * t: coordinate transformation
   * pCOS2COS1:  pose of COS2 relative to COS1 (expressed in COS1)
   * tCOS2COS1:  coordinate transformation from COS1 to COS2 (transforms representation of 3D coords in COS1 to
   * representation of same 3D points in COS2)
   * Robot: initial Robot coordinate system (initial pose of robot at initialization of SLAM robot)
   * Cam: camera coordinate system of corresponding robot (current pose of camera and robot - transformation between
   * current robot COS and camera COS is assumed to be identity)
   * World: world coordinate system, equivalent to initial robot coordinate system of robot with robot ID rID=1
   */

	/*	Eigen::Matrix4f pRobotWorld(Eigen::Matrix4f::Identity(4, 4));
	  Eigen::Matrix4f pWorldRobot(Eigen::Matrix4f::Identity(4, 4));
	  Eigen::Matrix4f tWorldRobot(Eigen::Matrix4f::Identity(4, 4));

	  typedef std::map<int, bool> ptClUpdatedMap;
	  typedef std::vector<int> rIDToUpdateVec;
	  typedef std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > > ptClMap;

	  // check robot ID for which ptcls can be updated
	  vector<int> rIDToUpdate;
	  for (ptClUpdatedMap::iterator it = this->ptClsInGlobalWorldCOSUpdated.begin(); it !=
	 this->ptClsInGlobalWorldCOSUpdated.end(); ++it) {
		  if (it->second == false) {
			  cout << "5-1: ==========> Now updateing the PtCls of  " << it->first << endl;
			  rIDToUpdate.push_back(it->first);
		  }
	  }

	  if (rIDToUpdate.empty()) { // all ptcls up-to-date
		  cout << "		All PtCls up-to-date" << endl;
	  }
	  else { // update ptcls
		  // update all ptcls from all robot IDs in rIDToUpdate
		  for (rIDToUpdateVec::iterator itVec = rIDToUpdate.begin(); itVec < rIDToUpdate.end(); ++itVec) {
			  // iterate through all robots that need their ptCls to be updated

			  for (ptClMap::iterator it = this->ptCls.begin(); it != this->ptCls.end(); ++it) {
				  // iterate through all ptcls
				  if ( (it->second.first == *itVec) ) {
					  // check if the corresponding robot needs update
					  this->ptClsInGlobalWorldCOS[it->first].first = it->second.first;			// ptCls ->
	 globalworldCOS

					  // pRobotWorld = this->pRobotsWorld[*itVec]; // copying the transformation pose

					  pRobotWorld = this->Pose_diff; // copying the transformation pose
					  tWorldRobot = pRobotWorld;
					  //	cout << "Update the point clouds with respect to this pose "  << endl;
					  cout << "5-2: ==========> Now updating the PtCls " << endl;
					  HelperFcts::invTrafo(pRobotWorld, pWorldRobot); // taking the inverse transform

					  //   pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);
					  pcl::transformPointCloud(it->second.second, this->ptClsInGlobalWorldCOS[it->first].second,
	 pRobotWorld); // transform the whole point cloud
				  }
			  }
			  this->ptClsInGlobalWorldCOSUpdated[*itVec] = true;
		  }
	  }*/
}
//
//
//
// NOT IN USEDDDDDDDD
//
//
//
void CentralStorage::estimateScale()
{
	cout << "3: ===========> Entering to estimate scale " << endl;

	// if query = robot 2
	// iGTrafo = pC1C2 = tC2C1
	int queryImgNbr = this->testImgCtr;
	int testImgNbr = matchedKeyFramesMap[queryImgNbr].second;
	Eigen::Matrix4f iGTrafo = this->iGMap[queryImgNbr].first;
	keyFrame queryKF, testKF;

	queryKF = this->kFrames[queryImgNbr];
	testKF = this->kFrames[testImgNbr];
	int queryRID = queryKF.rID;
	int testRID = testKF.rID;

	// Extract descriptors from imgs
	Mat queryImgDescr, testImgDescr;
	this->extractor->compute(queryKF.img, queryKF.KPts, queryImgDescr);
	this->extractor->compute(testKF.img, testKF.KPts, testImgDescr);

	// match descriptors
	std::vector<DMatch> resultingMatches; // OpenCV Descriptor match
	this->matcher->match(queryImgDescr, testImgDescr, resultingMatches);

	vector<Point2f> matched2DPtsQuery, matched2DPtsTest;
	vector<Eigen::Vector3f> matched3DPtsQuery, matched3DPtsTest;
	// vector<float> measurements;
	Eigen::Vector3f matched3DPtQuery(0.0, 0.0, 0.0), matched3DPtTest(0.0, 0.0, 0.0);

	vector<KeyPoint> matchedQueryKPts, matchedTestKPts;

	for (int i = 0; i < resultingMatches.size(); ++i)
	{
		matchedQueryKPts.push_back(queryKF.KPts[resultingMatches[i].queryIdx]);
		matchedTestKPts.push_back(testKF.KPts[resultingMatches[i].trainIdx]);
	}
	//	HelperFcts::displayImageKPts("queryImg", queryKF.img, matchedQueryKPts);
	//	HelperFcts::displayImageKPts("testImg", testKF.img, matchedTestKPts);
	//	HelperFcts::displayMatches("matches", queryKF.img, testKF.img, queryKF.KPts, testKF.KPts, resultingMatches);
	//	waitKey(10000);

	for (int i = 0; i < resultingMatches.size(); ++i)
	{
		Point2f matched2DPtQuery = queryKF.KPts[resultingMatches[i].queryIdx].pt;
		Point2f matched2DPtTest = testKF.KPts[resultingMatches[i].trainIdx].pt;

		HelperFcts::calc3DPt(matched2DPtQuery, queryKF.fx, queryKF.fy, queryKF.cx, queryKF.cy, queryKF.idepth,
							 matched3DPtQuery);
		// HelperFcts::transform3DPt(queryKF.camToRobot, matched3DPtQuery, matched3DPtQuery);

		HelperFcts::calc3DPt(matched2DPtTest, testKF.fx, testKF.fy, testKF.cx, testKF.cy, testKF.idepth, matched3DPtTest);
		// iGTrafo = pC1C2 = tC2C1
		// iGTrafo = pC2C1 = tC1C2
		Eigen::Matrix4f iGTrafoInv(Eigen::Matrix4f::Identity(4, 4));
		HelperFcts::invTrafo(iGTrafo, iGTrafoInv);
		HelperFcts::transform3DPt(iGTrafoInv, matched3DPtQuery, matched3DPtQuery);

		KalmanFilterScale::Vector u(0);
		KalmanFilterScale::Vector z(3);
		for (int i = 0; i < 3; ++i)
		{

			float measurement = matched3DPtTest(i) / matched3DPtQuery(i);
			z(i + 1) = measurement;
		}
		this->filter.step(u, z);
	}
	cout << "scale estimate:	" << this->filter.getX()(1) << endl
		 << "cov:	" << this->filter.calculateP()(1, 1)
		 << endl;
	this->robotScales[queryRID] = this->filter.getX()(1);
}

void CentralStorage::clearData()
{
	this->kFrames.clear();

	this->ptCls.clear();
	this->ptClsInGlobalWorldCOS.clear();
	this->ptClsInGlobalWorldCOSUpdated.clear();

	this->localPoses.clear();
	this->localScales.clear();
	this->robotScales.clear();

	this->pRobotsWorld.clear();
	this->localPoseGraphs.clear();

	globalPoseGraph.clear();

	this->testBOWDescriptors_allRobots.release();
	this->testImgCtr = 0;

	this->matches.clear();
	this->boolMatch = false;
	this->matchedKeyFrames = std::make_pair(-1, -1);
	this->matchedKeyFramesMap.clear();

	this->iGMap.clear();
}
