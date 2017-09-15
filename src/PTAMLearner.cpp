#include <fstream>
#include <typeinfo>
#include <cmath>
#include "Helper.h"
#include "PTAMLearner.h"
#include <ros/package.h>
#include <rl_nav/QPrediction.h>

using namespace std;

pthread_mutex_t PTAMLearner::gazeboModelState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PTAMLearner::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PTAMLearner::pixelCloud_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PTAMLearner::pose_mutex = PTHREAD_MUTEX_INITIALIZER;

PTAMLearner::PTAMLearner()
{
	qClient = nh.serviceClient<rl_nav::QPrediction>("/planner/qValue");
	pointCloud_sub = nh.subscribe("/vslam/frame_points", 100, &PTAMLearner::pointCloudCb, this);
	pixelCloud_sub = nh.subscribe("/vslam/frame_pixels", 100, &PTAMLearner::pixelCloudCb, this);
	gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &PTAMLearner::gazeboModelStatesCb, this);
	pose_sub = nh.subscribe("/vslam/pose_world",100, &PTAMLearner::poseCb, this);
	srand (time(NULL));
	lastBestQStateAction = nullTuple;
}

void PTAMLearner::gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr)
{
	pthread_mutex_lock(&gazeboModelState_mutex);	
	robotWorldPose = modelStatesPtr->pose.back();
	pthread_mutex_unlock(&gazeboModelState_mutex);
}

void PTAMLearner::pointCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr)	
{
	pthread_mutex_lock(&pointCloud_mutex);
	currentPointCloud = *pointCloudPtr;
	pthread_mutex_unlock(&pointCloud_mutex);
}

void PTAMLearner::pixelCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pixelCloudPtr)	
{
	pthread_mutex_lock(&pixelCloud_mutex);
	currentPixelCloud = *pixelCloudPtr;
	pthread_mutex_unlock(&pixelCloud_mutex);
}

void PTAMLearner::poseCb(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr)
{
	pthread_mutex_lock(&pose_mutex);
	ptamPose = *posePtr;
	pthread_mutex_unlock(&pose_mutex);
}


void PTAMLearner::getActions()
{
	if(!possibleTrajectories.size())
		for(auto trajectory : Helper::getTrajectories())
			possibleTrajectories.push_back(getAction(trajectory));
}

CommandStateActionQ PTAMLearner::getAction(vector<float> input)
{
	vector<float> rl_input;
	vector<int> q_input;
	pcl::PointCloud<pcl::PointXYZ> nextPointCloud = Helper::getPointCloudAtPosition(input), pointCloud, pixelCloud;
	float dir = input[12], del_heading = atan(input[5]), var;

	//RL params
	pthread_mutex_lock(&pointCloud_mutex);
	pointCloud = currentPointCloud;
	pthread_mutex_unlock(&pointCloud_mutex);

	pthread_mutex_lock(&pixelCloud_mutex);
	pixelCloud = currentPixelCloud;
	pthread_mutex_unlock(&pixelCloud_mutex);

	pthread_mutex_lock(&pose_mutex);
	var = ptamPose.pose.covariance[0] + ptamPose.pose.covariance[7] + ptamPose.pose.covariance[14] + ptamPose.pose.covariance[21] + ptamPose.pose.covariance[28] + ptamPose.pose.covariance[35]; 
	pthread_mutex_unlock(&pose_mutex);
	
	//rl_input.push_back(1);

	if(dir==1)
	{
		rl_input.push_back(1);
		rl_input.push_back(0);
	}
	else
	{
		rl_input.push_back(0);
		rl_input.push_back(1);
	}
	q_input.push_back((int)(dir==1)?1:0);
	
	
	if(fabs(del_heading)*180.0/PI > 30)
	{
		rl_input.push_back(1);
		q_input.push_back(19);
	}
	else
	{
		rl_input.push_back(fabs((del_heading)*180.0/(30*PI)));
		q_input.push_back((int) 19*fabs((del_heading)*180.0/(30*PI)));
	}
	
	rl_input.push_back(min(1,pointCloud.size()/600.0));

	vector<pcl::PointXYZ> commonPoints = Helper::pointCloudIntersection(pointCloud,nextPointCloud);
	rl_input.push_back(min(1,commonPoints.size()/600.0));
	q_input.push_back((int) min(19,commonPoints.size()/30.0));
	

	rl_input.push_back((isnan(var) or isinf(var))?1:var);


	/*float x_mean=0, y_mean=0, x_var=0, y_var=0;
	if(pixelCloud.points.size()!=0)
	{
		for(int i=0;i<pixelCloud.points.size();i++)
		{
			x_mean += pixelCloud.points[i].x;
			y_mean += pixelCloud.points[i].y;
		
			x_var += pixelCloud.points[i].x*pixelCloud.points[i].x;
			y_var += pixelCloud.points[i].y*pixelCloud.points[i].y;
		}
		
		x_mean /= pixelCloud.points.size();
		y_mean /= pixelCloud.points.size();
		
		x_var /= pixelCloud.points.size();
		y_var /= pixelCloud.points.size();
		
		x_var -= x_mean*x_mean;
		y_var -= y_mean*y_mean;
	}

	rl_input.push_back(x_mean);
	rl_input.push_back(y_mean);
	rl_input.push_back(x_var);
	rl_input.push_back(y_var);*/
/*
	rl_nav::QPrediction Q;
	Q.request.qInput.data = rl_input;
	qClient.call(Q);
	return make_tuple(input,rl_input,Q.response.qValue.data);*/

	return make_tuple(input,rl_input,getQ(q_input));
}


CommandStateActionQ PTAMLearner::getBestQStateAction(vector<float> lastCommand)
{
	if(lastBestQStateAction!=nullTuple)
		return lastBestQStateAction;
	
	vector<CommandStateActionQ> result;
	int index=-1;
	float maxQ = -numeric_limits<float>::infinity();  //init max Q to negative infinity
	float Q;
	
	getActions();
	
	for(auto input : possibleTrajectories)
	{	
		vector<float> inp = get<0>(input);
		if( lastCommand.size() and 
			!(lastCommand[12]+inp[12]) and 
			!(fabs(lastCommand[5]) - fabs(inp[5])))
			continue;
		result.push_back(input);
		Q = get<2>(input);
		if(maxQ<Q)
		{
			maxQ = Q;
			index = result.size()-1;
		}
	}
	
	if(maxQ == -numeric_limits<float>::infinity() or !result.size())
	{
		if(possibleTrajectories.size())
			lastBestQStateAction = possibleTrajectories[rand()%possibleTrajectories.size()];
		else
		{
			lastBestQStateAction = nullTuple;
			return getRandomStateAction();
		}
	}	

		
	else if(index!=-1 and result.size())
		lastBestQStateAction = result[index];
	
	return lastBestQStateAction;
}

CommandStateActionQ PTAMLearner::getEpsilonGreedyStateAction(float epsilon, vector<float> lastCommand)
{
	if((rand() % 100) < epsilon)
		return getBestQStateAction(lastCommand);
	else
		return getRandomStateAction();
}

CommandStateActionQ PTAMLearner::getRandomStateAction()
{
	vector<vector<float> > trajectories = Helper::getTrajectories();	
	return getAction(trajectories[rand()%trajectories.size()]);
}

CommandStateActionQ PTAMLearner::getThresholdedRandomStateAction(float qThreshold, int maxIters)
{
	CommandStateActionQ result;

	do 
	{
		result = getRandomStateAction();
		maxIters--;
	} while(get<2>(result) < qThreshold and maxIters>=0);

	return result;
}

//Thresholded aligning with global path
CommandStateActionQ PTAMLearner::getThresholdedClosestAngleStateAction(float qThreshold, float nextAngle, vector<float> lastCommand)
{
	vector<CommandStateActionQ> potentialInputs;
	getActions();
	
	for(auto input : possibleTrajectories)
		if(get<2>(input) > qThreshold)
			potentialInputs.push_back(input);
	 cout<<"potentialInputs size: "<<potentialInputs.size()<<endl;
	if(potentialInputs.size()<1)
		return getBestQStateAction(lastCommand);
	else
	{
		CommandStateActionQ result = potentialInputs[rand()%potentialInputs.size()];
		float currentAngle = Helper::getQuatOrientation(robotWorldPose.orientation)[2], min_diff = numeric_limits<float>::infinity(), angle_diff;
		float maxQ = -numeric_limits<float>::infinity();
		for(auto input : potentialInputs)
		{	
			vector<float> command = get<0>(input);
			angle_diff = fabs(nextAngle - (currentAngle + command[12]*atan(command[5])));
			if(angle_diff<min_diff or (angle_diff == min_diff and maxQ<get<2>(input)))
			{
				min_diff = angle_diff;
				result = input;
				maxQ = get<2>(input);
			}

		}
		return result;
	}
}

void PTAMLearner::clear()
{
	possibleTrajectories.clear();
	lastBestQStateAction = nullTuple;
}