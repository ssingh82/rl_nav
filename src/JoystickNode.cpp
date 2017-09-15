#include <algorithm>
#include <limits>
#include <numeric>
#include <functional>
#include <thread>

#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstdio>

#include "Helper.h"
#include "JoystickNode.h"

#include <tf/transform_datatypes.h>

#include <rl_nav/ExpectedPath.h>

using namespace std;

pthread_mutex_t JoystickNode::pose_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::ptamInfo_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::plannerStatus_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::gazeboModelState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::globalPlanner_mutex = PTHREAD_MUTEX_INITIALIZER;
//tf::TransformBroadcaster JoystickNode::tfBroadcaster;

JoystickNode::JoystickNode()
{
	srand (time(NULL));
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1);
	//vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	planner_pub = nh.advertise<std_msgs::Float32MultiArray>("/planner/input",1);
	global_planner_pub = nh.advertise<std_msgs::Empty>("/planner/input/global",1);
	planner_reset_pub = nh.advertise<std_msgs::Empty>("/planner/reset",1);
	gazebo_state_reset_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1);
	ptam_com_pub = nh.advertise<std_msgs::String>("/vslam/key_pressed",1);
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/my_pose",1);
	next_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/my_next_pose",1);
	expected_pub = nh.advertise<geometry_msgs::PoseStamped>("/expected_pose",1);
	ptam_path_pub = nh.advertise<visualization_msgs::Marker>("/vslam_path",1);
	ptam_pc_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/vslam_pc",1);
	gazebo_path_pub = nh.advertise<visualization_msgs::Marker>("/gazebo_path",1);
	gazebo_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/gazebo_pose",1);
	init_pub = nh.advertise<std_msgs::Empty>("/rl/init",1);
	
	sendCommand_pub = nh.advertise<std_msgs::Empty>("/rl/sendCommand",1);
	
	safe_traj_pub = nh.advertise<std_msgs::Float32MultiArray>("/rl/safe_trajectories",1);
	unsafe_traj_pub = nh.advertise<std_msgs::Float32MultiArray>("/rl/unsafe_trajectories",1);
	ptam_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/vslam/world_pose",1);

	expectedPathClient = nh.serviceClient<rl_nav::ExpectedPath>("/planner/global/expected_path");
	
	ifstream ratioFile("ratioFile.txt");
	ratioFile >> rlRatio >> num_episodes;
	num_episodes = 0;
	if(!rlRatio)
		rlRatio = 10;
	ratioFile.close();
	episodeList = Helper::readFeatureExpectation("tempfeFile.txt");
	num_episodes = episodeList.size();
	state = 0;
	breakCount = 0;
	joy.buttons = vector<int>(11,0);
	joy.axes = vector<float>(8,0);
	prevQ = 1;	
	initState.model_name = "";
	just_init = true;
	initialized = true;
	initY = 0;
	num_broken = 0;
	Q_THRESH = 0;
	up = down = left = right = true;
	vel_scale = 1.0;
	num_steps = 0;	
	
	qFile.open(string("qData.txt"),ios::app);
	
	joy_sub = nh.subscribe("/joy", 100, &JoystickNode::joyCb, this);
	init_sub = nh.subscribe("/rl/init", 100, &JoystickNode::initCb, this);
	//initDone_sub = nh.subscribe("/rl/initDone", 100, &JoystickNode::initDoneCb, this);
	sendCommand_sub = nh.subscribe("/rl/sendCommand", 100, &JoystickNode::sendCommandCb, this);
	pose_sub = nh.subscribe("/vslam/pose_world",100, &JoystickNode::poseCb, this);
	cam_pose_sub = nh.subscribe("/vslam/pose",100, &JoystickNode::camPoseCb, this);
	pointCloud_sub = nh.subscribe("/vslam/pc2", 100, &JoystickNode::pointCloudCb, this);
	ptamInfo_sub = nh.subscribe("/vslam/info", 100, &JoystickNode::ptamInfoCb, this);
	ptamStart_sub = nh.subscribe("/vslam/started", 100, &JoystickNode::ptamStartedCb, this);
	plannerStatus_sub = nh.subscribe("/planner/status", 100, &JoystickNode::plannerStatusCb, this);
	globalPoints_sub = nh.subscribe("/planner/global/path", 100, &JoystickNode::globalNextPoseCb, this);
	gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &JoystickNode::gazeboModelStatesCb, this);
	waypoint_sub = nh.subscribe("/move_base_simple/waypoint",100,&JoystickNode::waypointCb,this);

	vslam_path.id=0;
	vslam_path.lifetime=ros::Duration(1);
	vslam_path.header.frame_id = "/world";
	vslam_path.header.stamp = ros::Time::now();
	vslam_path.ns = "pointcloud_publisher";
	vslam_path.action = visualization_msgs::Marker::ADD;
	vslam_path.type = visualization_msgs::Marker::LINE_STRIP;
	vslam_path.color.r=0.0;
	vslam_path.color.b=1.0;
	vslam_path.color.a=1.0;
	vslam_path.scale.x=0.1;
	vslam_path.pose.orientation.w=1.0;

	gazebo_path.id=0;
	gazebo_path.lifetime=ros::Duration(1);
	gazebo_path.header.frame_id = "/world";
	gazebo_path.header.stamp = ros::Time::now();
	gazebo_path.ns = "pointcloud_publisher";
	gazebo_path.action = visualization_msgs::Marker::ADD;
	gazebo_path.type = visualization_msgs::Marker::LINE_STRIP;
	gazebo_path.color.r=1.0;
	gazebo_path.color.g=0.0;
	gazebo_path.color.a=1.0;
	gazebo_path.scale.x=0.1;
	gazebo_path.pose.orientation.w=1.0;

	initState.reference_frame = "world";
	initState.pose.position.z = 0;

	MAP=-1;
	ros::NodeHandle p_nh("~");
	p_nh.getParam("qThresh", Q_THRESH);
	p_nh.getParam("mode", MODE);
	p_nh.getParam("num_episodes", MAX_EPISODES);
	p_nh.getParam("max_steps", MAX_STEPS);
	p_nh.getParam("init_x", initState.pose.position.x);
	p_nh.getParam("init_y", initState.pose.position.y);
	p_nh.getParam("init_Y", initYaw);
	p_nh.getParam("map", MAP);
	p_nh.getParam("vel_scale", vel_scale);
	p_nh.getParam("predictor", PREDICTOR);
	initState.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, initYaw);
	learner.clear();
	if(!MODE.compare("TRAIN") or !MODE.compare("TEST"))
	{
		state = 1;
		init_pub.publish(std_msgs::Empty());
	}
	else if(!MODE.compare("MAP"))
		state = 2;
}

JoystickNode::~JoystickNode()
{
	qFile.close();
	ros::NodeHandle p_nh("~");
	p_nh.deleteParam("qThresh");
	p_nh.deleteParam("mode");
	p_nh.deleteParam("num_episodes");
	p_nh.deleteParam("max_steps");
	p_nh.deleteParam("init_x");
	p_nh.deleteParam("init_y");
	p_nh.deleteParam("init_Y");
	p_nh.deleteParam("map");
	p_nh.deleteParam("vel_scale");
	p_nh.deleteParam("predictor");

	ofstream ratioFile("ratioFile.txt");
	ratioFile << ((rlRatio==90)?10:rlRatio) << " " << 0 << endl;
	ratioFile.close();
}

void JoystickNode::ptamStartedCb(const std_msgs::EmptyPtr emptyPtr)
{
	episode.clear();
	breakCount = 0;
	init_pub.publish(std_msgs::Empty());
}

/**
 *	PTAM Initializer callback
 */
void JoystickNode::initCb(const std_msgs::EmptyPtr emptyPtr)
{

planner_reset_pub.publish(std_msgs::Empty());//stop planner
gazebo_path.points.clear();
vslam_path.points.clear();
initY = 0;
pointCloud.points.clear();
pointCloud.height=0;
pointCloud.width=0;
	if(!MODE.compare("TRAIN") or !MODE.compare("TEST"))
	{
		initState.pose.position.x = -rand()%5 - 1;
		initState.pose.position.y = rand()%3 + 0.5;
		initState.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, (rand()%360)*PI/180);
	}
	std_msgs::String resetString, spaceString;
	geometry_msgs::Twist twist;
	
	resetString.data = "r";
	spaceString.data = "Space";
	ptam_com_pub.publish(resetString);
	ptam_com_pub.publish(resetString);
	ptam_com_pub.publish(resetString);
	ptam_com_pub.publish(resetString);
	
	gazebo_state_reset_pub.publish(initState);
	ptam_com_pub.publish(spaceString);

	twist.linear.x=-0.4;
	clock_t t = clock();
	ros::Rate r(10);
	while(((float) (clock() - t))/CLOCKS_PER_SEC < 0.15)	
	{
		vel_pub.publish(twist);
		r.sleep();
	}
	
	num_broken = 0;	
	just_init=true;
	initialized = false;
	ros::Rate(1).sleep();
	ptam_com_pub.publish(spaceString);
	ros::Rate(2).sleep();
	//initDone_pub.publish(std_msgs::Empty());

}
  
/**
 *	Receive PTAM pose of camera in world
 */
void JoystickNode::poseCb(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr)
{
	pthread_mutex_lock(&pose_mutex);
	pose = *posePtr;
	pose.header.frame_id = "world";
	if(just_init)
	{
		just_init=false;
		vector<double> orientation = Helper::getQuatOrientation(pose.pose.pose.orientation);
		double angle = abs(orientation[0]);
		//if((orientation[0]-3.14)*(orientation[0]-3.14) > 0.003)
		if((angle-3.1415926)*(angle-3.1415926) > 0.003)
			init_pub.publish(std_msgs::Empty());
		else
		{
			just_init = false;
			initialized = true;
			gazebo_path.points.clear();
			vslam_path.points.clear();
			startRobotPose = robotWorldPose;
			startPTAMPose = pose.pose.pose;
			if(state==1)
				sendCommand_pub.publish(std_msgs::Empty());
			/*else if(state==2)
				global_planner_pub.publish(std_msgs::Empty());*/
		}
	}	
	else
		initialized = true;
	if(!initY)
		initY = pose.pose.pose.position.y;
	//float trace = pose.pose.covariance[0] + pose.pose.covariance[7] + pose.pose.covariance[14] + pose.pose.covariance[21] + pose.pose.covariance[28] + pose.pose.covariance[35];
	//if(sqrt(inner_product(pose.pose.covariance.begin(), pose.pose.covariance.end(), pose.pose.covariance.begin(), 0.0)) > 0.03)
	//if(trace > 0.03)
	if((initY - pose.pose.pose.position.y)*(initY - pose.pose.pose.position.y) >=0.15)
		num_broken++;
	else if(num_broken>0)
		num_broken--;
	
	geometry_msgs::PoseStamped ps;
	ps.header = pose.header;
	ps.header.frame_id="world";
	ps.pose = pose.pose.pose;
	ptam_pose_pub.publish(ps);

	ps.pose.position.z = 0;
	//if(MAP==1 or MAP==5)
	if(MAP==2 or MAP==3 or MAP==4 or MAP==-1)
	{
		ps.pose.position.x = pose.pose.pose.position.z;// + startPTAMPose.position.z + startRobotPose.position.y;
		ps.pose.position.y = -pose.pose.pose.position.x;// - startPTAMPose.position.x + startRobotPose.position.x;
		vector<double> curr_angles = Helper::getQuatOrientation(pose.pose.pose.orientation);
		/*vector<double> ptam_init_angles = Helper::getQuatOrientation(startPTAMPose.orientation);
		vector<double> robot_init_angles = Helper::getQuatOrientation(startRobotPose.orientation);*/
		if(curr_angles[2]*curr_angles[2] < (PI - fabs(curr_angles[2]))*(PI - fabs(curr_angles[2])))
			ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, PI/2 +  curr_angles[1]);// - ptam_init_angles[1]+robot_init_angles[2]);
		else
			ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -PI/2 - curr_angles[1]);// - ptam_init_angles[1]+robot_init_angles[2]);
	}
	else //if(MAP==2 or MAP==3 or MAP==4)
	{
		ps.pose.position.y = -pose.pose.pose.position.x;// + startPTAMPose.position.z + startRobotPose.position.y;
		ps.pose.position.x = -pose.pose.pose.position.z;// - startPTAMPose.position.x + startRobotPose.position.x;
		vector<double> curr_angles = Helper::getQuatOrientation(pose.pose.pose.orientation);
		/*vector<double> ptam_init_angles = Helper::getQuatOrientation(startPTAMPose.orientation);
		vector<double> robot_init_angles = Helper::getQuatOrientation(startRobotPose.orientation);*/
		if(curr_angles[2]*curr_angles[2] < (PI - fabs(curr_angles[2]))*(PI - fabs(curr_angles[2])))
			ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, curr_angles[1]);// - ptam_init_angles[1]+robot_init_angles[2]);
		else
			ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -PI/2 + curr_angles[1]);// - ptam_init_angles[1]+robot_init_angles[2]);
	}

	pose_pub.publish(ps);
	
	//vslam_path.points.push_back(pose.pose.pose.position);
	vslam_path.points.push_back(ps.pose.position);
	
	ptam_path_pub.publish(vslam_path);

	ps.pose.position.z = 0;//pose.pose.pose.position.y;
	ps.pose.position.x = robotWorldPose.position.x;//(-robotWorldPose.position.x + startPTAMPose.position.z + startRobotPose.position.x);
	ps.pose.position.y = robotWorldPose.position.y;//(-robotWorldPose.position.y + startPTAMPose.position.x + startRobotPose.position.y);
	gazebo_path.points.push_back(ps.pose.position);
	gazebo_path_pub.publish(gazebo_path);

	pthread_mutex_unlock(&pose_mutex);
}

void JoystickNode::camPoseCb(const geometry_msgs::PoseWithCovarianceStampedPtr camPosePtr)
{
	//pcl::PointCloud<pcl::PointXYZRGB> pc = pointCloud;
	//pc.pose = camPosePtr->pose.pose;
	//ptam_pc_pub.publish(pc);
}

/**
 *	Receive next expected robot pose w.r.t. current pose along global path
 */
void JoystickNode::globalNextPoseCb(const std_msgs::Float32MultiArrayPtr arrayPtr)
{
	pthread_mutex_lock(&globalPlanner_mutex);
	vector<float> input = arrayPtr->data;
	
	CommandStateActionQ step = learner.getAction(input);
	float Q = get<2>(step);
	
	//vector<float> stateAction = get<1>(step);
	
	geometry_msgs::PoseStamped ps;
	ps = Helper::getPoseFromInput(input, pose);
	expected_pub.publish(ps);
	ptam_com::ptam_info info;
		
	pthread_mutex_lock(&ptamInfo_mutex);
	info = ptamInfo;
	pthread_mutex_unlock(&ptamInfo_mutex);
	
	state = 1;
	//if((robotWorldPose.position.x - goal[0])*(robotWorldPose.position.x - goal[0]) + (robotWorldPose.position.y - goal[1])*(robotWorldPose.position.y - goal[1]) <= 0.25
	//if(num_broken>3 or !info.trackingQuality)
	if(!info.trackingQuality)
		planner_reset_pub.publish(std_msgs::Empty());//stop planner
	bool possible_break = (Q < Q_THRESH);
	cout<<"Q of step is "<<Q<<endl;
	/*if(!PREDICTOR.compare("RL"))
		possible_break = (Q < Q_THRESH);
	else if(!PREDICTOR.compare("SVM"))
		possible_break = learner.predict(stateAction);
*/
	if(!MODE.compare("MAP") and possible_break)
	{
		cout<<"predicted break "<< Q<<endl;
		for(auto i: get<1>(step))
			cout<<i<<'\t'; 
		cout<<endl;
		planner_reset_pub.publish(std_msgs::Empty());//stop planner
		planner_reset_pub.publish(std_msgs::Empty());//stop planner
		learner.clear();
		sendCommand_pub.publish(std_msgs::Empty());
	}
	
	pthread_mutex_unlock(&globalPlanner_mutex);
}

/**
 *	Receive joystick input
 */
void JoystickNode::joyCb(const sensor_msgs::JoyPtr joyPtr)
{
	geometry_msgs::Twist command;
	if(joyPtr->buttons[POWER] and !joy.buttons[POWER])
		cout << breakCount << " " << rlRatio << " " << num_steps << " " << num_episodes <<endl;
	if((joyPtr->axes[DH] and !joy.axes[DH]) and joyPtr->axes[DH] != joy.axes[DH])
	{
		if(joyPtr->axes[DH]==-1)
		{
			Helper::right = not Helper::right;
			if(Helper::left == false and Helper::right == false)
				Helper::left = true;
		}
		else if(joyPtr->axes[DH]==1)
		{
			Helper::left = not Helper::left;
			if(Helper::left == false and Helper::right == false)
				Helper::right = true;
		}
	}
	else if((joyPtr->axes[DV] and !joy.axes[DV]) and joyPtr->axes[DV] != joy.axes[DV])
	{
		if(joyPtr->axes[DV]==1)
		{
			Helper::up = not Helper::up;
			if(Helper::up == false and Helper::down == false)
				Helper::down = true;
		}
		else if(joyPtr->axes[DV]==-1)
		{
			Helper::down = not Helper::down;
			if(Helper::up == false and Helper::down == false)
				Helper::up = true;
		}
	}
	else if(joyPtr->buttons[RB] and !joy.buttons[RB])
	{
		std_msgs::String resetString;
		resetString.data = "r";
		ptam_com_pub.publish(resetString);
	}
	else if(joyPtr->buttons[LB] and !joy.buttons[LB])
	{
		std_msgs::String spaceString;
		spaceString.data = "Space";
		ptam_com_pub.publish(spaceString);
	}
	else if(joyPtr->buttons[START] and !joy.buttons[START])
		init_pub.publish(std_msgs::Empty());
	else if(joyPtr->buttons[BACK] and !joy.buttons[BACK])
		ros::shutdown();
	else if(joyPtr->buttons[A] and !state)
	{
		//send input to planner
		state = 1;
		sendCommand_pub.publish(std_msgs::Empty());
	}
	else if(joyPtr->buttons[B]  and !joy.buttons[B])
	{
		if(state)
			planner_reset_pub.publish(std_msgs::Empty());//stop planner
		state = 0;
		num_broken = 0;
		breakCount = 0;
	}
	else if(joyPtr->buttons[Y])
	{
		num_broken = 0;
		breakCount = 0;
	}
	else if(joyPtr->buttons[X] and !joy.buttons[X])
	{
		state = 2;
		global_planner_pub.publish(std_msgs::Empty());
	}
	else if((fabs(joyPtr->axes[LV])>0.009 or fabs(joyPtr->axes[RH])>0.009) and !state)
	{
		command.linear.x = joyPtr->axes[LV]*vel_scale;
		command.angular.z = joyPtr->axes[RH]*vel_scale;
		vel_pub.publish(command);
	}
	joy = *joyPtr;
}

/**
 *	Receive PTAM point cloud
 */
void JoystickNode::pointCloudCb(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr)	
{
	pthread_mutex_lock(&pointCloud_mutex);
	pointCloud = *pointCloudPtr;
	pthread_mutex_unlock(&pointCloud_mutex);
}

/**
 *	Receive PTAM Info
 */
void JoystickNode::ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr)	
{
	pthread_mutex_lock(&ptamInfo_mutex);
	ptamInfo = *ptamInfoPtr;
	pthread_mutex_unlock(&ptamInfo_mutex);
}

/**
 *	Receive Gazebo Models data
 */
void JoystickNode::gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr)
{
	pthread_mutex_lock(&gazeboModelState_mutex);	
	robotWorldPose = modelStatesPtr->pose.back();
	if(just_init and not initState.model_name.length() and modelStatesPtr->name.back().length())
		init_pub.publish(std_msgs::Empty());
	initState.model_name = modelStatesPtr->name.back();
	geometry_msgs::PoseStamped ps;
	ps.header.stamp = ros::Time::now();
	ps.header.frame_id = "world";
	ps.pose = robotWorldPose;
	gazebo_pose_pub.publish(ps);
	pthread_mutex_unlock(&gazeboModelState_mutex);
}

/**
 *	Receive Planner status after executing local action
 */
void JoystickNode::plannerStatusCb(const std_msgs::StringPtr plannerStatusPtr)
{
	pthread_mutex_lock(&plannerStatus_mutex);
	
	if(!(plannerStatusPtr->data.compare("DONE")))
	{
		breakCount++;
		ptam_com::ptam_info info;
		
		pthread_mutex_lock(&ptamInfo_mutex);
		info = ptamInfo;
		pthread_mutex_unlock(&ptamInfo_mutex);
		//if(!episode.size() or (episode.size() and lastRLInput!=episode.back()))
		//{
		/*if(lastRLInput.size()==3)
			lastRLInput.push_back((num_broken <= 3 and info.trackingQuality)?0:1);*/
		for(auto i: lastRLInput)
			qFile<<i<<'\t';
		float var;
		pthread_mutex_lock(&pose_mutex);
		var = pose.pose.covariance[0] + pose.pose.covariance[7] + pose.pose.covariance[14] + pose.pose.covariance[21] + pose.pose.covariance[28] + pose.pose.covariance[35];
		pthread_mutex_unlock(&pose_mutex);
		qFile << ((isnan(var) or isinf(var))?1:var) << '\t' << ((num_broken <= 3 and info.trackingQuality)?0:1) << "\t;";
		
		/*for(auto input : Helper::getTrajectories())
		{
			vector<float> rlInput;
			tie(ignore, rlInput, ignore) = learner.getAction(input);
			for(auto i : rlInput)
				qFile<<i<<'\t';
			qFile<<';';
		}*/
		qFile << '\n';
		/*if(lastRLInput.size() != learner.NUM_FEATURES_Q)
			lastRLInput.resize(learner.NUM_FEATURES_Q);*/
		//lastRLInput.push_back(((isnan(var) or isinf(var))?1:var));
		lastRLInput.push_back(((num_broken <= 3 and info.trackingQuality)?0:1));
		episode.push_back(lastRLInput);
		//}
		// if(!MODE.compare("TRAIN"))
		// 	learner.updateQ(lastRLInput,get<1>(learner.getBestQStateAction(lastCommand)));
		if(num_broken>3 or !info.trackingQuality) 
		{	
			cout<<"Breaking after "<<breakCount<< " steps due to action with Q value "<< prevQ<<'\t';
			for(auto i: lastRLInput)
				cout<<i<<'\t'; 
			cout<< endl;
			breakCount = 0;
			initialized = false;
			Helper::saveTempFeatureExpectation(episode, "tempfeFile.txt");
			episodeList.push_back(episode);
			episode.clear();
			num_episodes++;

			if(MODE.compare("MAP") and num_episodes == MAX_EPISODES)// or num_steps >= MAX_STEPS)
			{
				if(!MODE.compare("TRAIN"))
				{
					//learner.episodeUpdate(episodeList);
					//episodeList.clear();
					rlRatio+=10;
					num_steps = 0;
					num_episodes = 0;
					if(rlRatio==90)
					{
						//MODE = "TEST";
						ros::shutdown();
					}
				}
				else if(!MODE.compare("TEST"))
				{
					Helper::saveFeatureExpectation(episodeList, "feFile.txt");
					remove("tempfeFile.txt");
					episodeList.clear();
					episode.clear();
					ros::shutdown();
				}
			}
			if(!MODE.compare("MAP"))
				planner_reset_pub.publish(std_msgs::Empty());//stop planner
			/*else
				init_pub.publish(std_msgs::Empty());*/
		
		}
		else if(state==1)
		{
			if(!MODE.compare("MAP"))
			{
				state = 2;
				breakCount = 0;
				global_planner_pub.publish(std_msgs::Empty());
			}
			else
				sendCommand_pub.publish(std_msgs::Empty());
			
		}
	}
	
	pthread_mutex_unlock(&plannerStatus_mutex);

}

void JoystickNode::waypointCb(const geometry_msgs::PoseStampedPtr waypointPosePtr)
{
	waypointPose = waypointPosePtr->pose;
}

/**
 *	Sending commands to the robot 
 */
void JoystickNode::sendCommandCb(std_msgs::EmptyPtr emptyPtr)
{
	if(initialized)
	{
		std_msgs::Float32MultiArray planner_input, trajectories;
/*		vector<float> safe_inputs, unsafe_inputs;
		float Q;
		int safe = 0, unsafe = 0;
	
		for(auto input: Helper::getTrajectories())
		{
			tie(ignore, ignore, Q) = learner.getAction(input);
			if(Q>Q_THRESH)
			{
				safe_inputs.insert(safe_inputs.end(), input.begin(), input.end());
				safe++;
			}
			else
			{
				unsafe_inputs.insert(unsafe_inputs.end(), input.begin(), input.end());
				unsafe++;
			}
		}
		safe_inputs.push_back(safe);
		unsafe_inputs.push_back(unsafe);
		trajectories.data = safe_inputs;
		safe_traj_pub.publish(trajectories);
		trajectories.data = unsafe_inputs;
		unsafe_traj_pub.publish(trajectories);*/
		//incremental training epsilon greedy
		if(!MODE.compare("TRAIN"))
			tie(lastCommand, lastRLInput, prevQ) = learner.getRandomStateAction();//learner.getEpsilonGreedyStateAction(rlRatio,lastCommand);
		else if(!MODE.compare("TEST"))
			tie(lastCommand, lastRLInput, prevQ) = learner.getEpsilonGreedyStateAction(95,lastCommand);
		else //if(!MODE.compare("MAP"))
		{
			rl_nav::ExpectedPath expectedPath;
			expectedPathClient.call(expectedPath);
			vector<float> poses = expectedPath.response.expectedPath.data;
			float nextAngle = atan(poses[5]);
			
			//float nextAngle = atan2 (waypointPose.position.y + pose.pose.pose.position.x, waypointPose.position.x + pose.pose.pose.position.z);
			
			// float nextAngle = Helper::getQuatOrientation(waypointPose.orientation)[2];

			//if(!PREDICTOR.compare("RL"))
				tie(lastCommand, lastRLInput, prevQ) = learner.getThresholdedClosestAngleStateAction(Q_THRESH, nextAngle, lastCommand);
			/*else if(!PREDICTOR.compare("SVM"))
				tie(lastCommand, lastRLInput, prevQ) = learner.getSLClosestAngleStateAction(nextAngle);*/
			//tie(lastCommand, lastRLInput, prevQ) = learner.getRandomStateAction();
		
		}	
		num_steps++;
		//next_pose_pub.publish(Helper::getPoseFromInput(lastCommand, pose));
		
		planner_input.data = lastCommand;
		//learner.clear();
//		planner_pub.publish(planner_input);//send input to planner
	}	
}
