#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <utility>
#include <tuple>

#include "PTAMLearner.h"

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <ptam_com/ptam_info.h>
#include <pcl_ros/point_cloud.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <visualization_msgs/Marker.h>

using namespace std;

class JoystickNode
{
private:
	enum {A, B, X, Y, LB, RB, BACK, START, POWER, LS, RS}; //joystick buttons according to their index
	enum {LH, LV, LT, RH, RV, RT, DH, DV}; //joystick axes
	
	ros::NodeHandle nh;

	//Subscribers
	ros::Subscriber joy_sub,
					pose_sub,
					pointCloud_sub,
					ptamInfo_sub,
					plannerStatus_sub,
					gazeboModelStates_sub,
					globalPoints_sub,
					init_sub,
					initDone_sub,
					sendCommand_sub,
					ptamStart_sub,
					cam_pose_sub,
					waypoint_sub;

	// Publishers
	ros::Publisher  vel_pub,
					planner_pub,
					planner_reset_pub,
					gazebo_state_reset_pub,
					ptam_com_pub,
					pose_pub,
					next_pose_pub,
					global_planner_pub,
					expected_pub,
					ptam_path_pub,
					gazebo_path_pub,
					init_pub,
					initDone_pub,
					sendCommand_pub,
					start_pub,
					safe_traj_pub,
					unsafe_traj_pub,
					gazebo_pose_pub,
					ptam_pose_pub,
					ptam_pc_pub;

	ros::ServiceClient expectedPathClient;
	
	//Mutex locks	
	static pthread_mutex_t pose_mutex;
	static pthread_mutex_t pointCloud_mutex;
	static pthread_mutex_t ptamInfo_mutex;
	static pthread_mutex_t plannerStatus_mutex;
	static pthread_mutex_t gazeboModelState_mutex;
	static pthread_mutex_t globalPlanner_mutex;
	
	geometry_msgs::PoseWithCovarianceStamped pose;
	geometry_msgs::Twist vel;
	pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
	ptam_com::ptam_info ptamInfo;
	sensor_msgs::Joy joy;
	gazebo_msgs::ModelState initState;
	geometry_msgs::Pose robotWorldPose;
	visualization_msgs::Marker vslam_path, gazebo_path;
	geometry_msgs::Pose startPTAMPose, startRobotPose, waypointPose;

	tf::TransformBroadcaster tfBroadcaster;
		
	string MODE, PREDICTOR;
	int MAX_EPISODES, MAX_STEPS, MAP;
	float Q_THRESH;
	int state, breakCount, num_broken, num_steps, num_episodes;
	vector<float> lastCommand; //last command sent to the planner
	vector<float> lastRLInput; //last RL Input
	vector<vector<float> > episode;
	vector<vector<vector<float> > > episodeList;
	bool badEstimate, just_init, initialized;
	float rlRatio;
	float prevQ; 	
	float initY, initX, initZ, initYaw, startX, startY, startYaw;
	ofstream qFile;
	bool left, right, up, down;
	float vel_scale;

	PTAMLearner learner; //Q learning agent
	
	//Callback Funtions
	void poseCb(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr);
	void camPoseCb(const geometry_msgs::PoseWithCovarianceStampedPtr camPosePtr);
	void joyCb(const sensor_msgs::JoyPtr joyPtr);
	void pointCloudCb(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr);	
	void ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr);	
	void plannerStatusCb(const std_msgs::StringPtr plannerStatusPtr);	
	void gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr);
	void globalNextPoseCb(const std_msgs::Float32MultiArrayPtr arrayPtr);
	void initCb(const std_msgs::EmptyPtr emptyPtr);
	void initDoneCb(const std_msgs::EmptyPtr emptyPtr);
	void sendCommandCb(const std_msgs::EmptyPtr emptyPtr);
	void ptamStartedCb(const std_msgs::EmptyPtr emptyPtr);
	void waypointCb(const geometry_msgs::PoseStampedPtr waypointPosePtr);

public:
	JoystickNode();
	~JoystickNode();
};