#include <vector>
#include <iostream>
#include <utility>
#include <tuple>
#include <cmath>

#include <gazebo_msgs/ModelStates.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "SarsaLearner.h"

using namespace std;

typedef tuple<vector<float>, vector<float>, float> CommandStateActionQ;
static CommandStateActionQ nullTuple = make_tuple(vector<float>(), vector<float>(), -numeric_limits<float>::infinity());

class PTAMLearner : //public SGDLearner
public SarsaLearner//, public SupervisedLearner
{
private:
	ros::NodeHandle nh;
	ros::Subscriber gazeboModelStates_sub, pointCloud_sub, pixelCloud_sub, pose_sub;
	ros::ServiceClient qClient;
	static pthread_mutex_t gazeboModelState_mutex, pointCloud_mutex, pixelCloud_mutex, pose_mutex;

	geometry_msgs::Pose robotWorldPose;
	pcl::PointCloud<pcl::PointXYZ> currentPointCloud, currentPixelCloud;
	geometry_msgs::PoseWithCovarianceStamped ptamPose;
	vector<CommandStateActionQ> possibleTrajectories;
	CommandStateActionQ lastBestQStateAction;
	void gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr);
	void pointCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr);
	void pixelCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pixelCloudPtr);
	void poseCb(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr);

public:	
	PTAMLearner();

	CommandStateActionQ getAction(vector<float> input);
	CommandStateActionQ getRandomStateAction();
	CommandStateActionQ getThresholdedRandomStateAction(float qThreshold, int maxIters);
	CommandStateActionQ getBestQStateAction(vector<float> lastCommand);
	CommandStateActionQ getEpsilonGreedyStateAction(float epsilon, vector<float> lastCommand);
	CommandStateActionQ getThresholdedClosestAngleStateAction(float qThreshold, float nextAngle, vector<float> lastCommand);
	void clear();
	void getActions();
};	