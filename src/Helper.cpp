#include <iostream>
#include <fstream>
#include <sstream>
#include "Helper.h"

#include "ptam_com/PosePointCloud.h"

using namespace std;

pthread_mutex_t Helper::pose_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::gazeboModelState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;

geometry_msgs::PoseWithCovarianceStamped Helper::pose;
ptam_com::ptam_info Helper::ptamInfo;
geometry_msgs::Pose Helper::robotWorldPose;
pcl::PointCloud<pcl::PointXYZ> Helper::currentPointCloud;
ros::ServiceClient Helper::posePointCloudClient;
int Helper::MAP, Helper::NUM_ACTIONS;
bool Helper::up, Helper::down, Helper::left, Helper::right;

Helper::Helper()
{
	posePointCloudClient = nh.serviceClient<ptam_com::PosePointCloud>("/vslam/posepointcloud");
	//posePointCloudClient = nh.serviceClient<ORB_SLAM::PosePointCloud>("/ORB_SLAM/PosePC");
	pose_sub = nh.subscribe("/vslam/pose_world",100, &Helper::poseCb, this);
	info_sub = nh.subscribe("/vslam/info",100, &Helper::ptamInfoCb, this);
	pointCloud_sub = nh.subscribe("/vslam/frame_points", 100, &Helper::pointCloudCb, this);
	gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &Helper::gazeboModelStatesCb, this);
	MAP=-1;
	NUM_ACTIONS = 14;
	up = down = left = right = true;
	ros::NodeHandle p_nh("~");
	p_nh.getParam("map", MAP);
	p_nh.getParam("num_actions", NUM_ACTIONS);
}

void Helper::poseCb(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr)
{
	pthread_mutex_lock(&pose_mutex);
	pose = *posePtr;
	pthread_mutex_unlock(&pose_mutex);
}

void Helper::ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr)
{
	pthread_mutex_lock(&info_mutex);
	ptamInfo = *ptamInfoPtr;
	pthread_mutex_unlock(&info_mutex);
}

void Helper::gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr)
{
	pthread_mutex_lock(&gazeboModelState_mutex);	
	robotWorldPose = modelStatesPtr->pose.back();
	pthread_mutex_unlock(&gazeboModelState_mutex);
}

void Helper::pointCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr)	
{
	pthread_mutex_lock(&pointCloud_mutex);
	currentPointCloud = *pointCloudPtr;
	pthread_mutex_unlock(&pointCloud_mutex);
}

pcl::PointCloud<pcl::PointXYZ> Helper::getPointCloudAtPosition(vector<float> input)
{
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	ptam_com::PosePointCloud posePointCloud;
	
	//PoseStamped from the new point
	pthread_mutex_lock(&pose_mutex);
	posePointCloud.request.pose = getPoseFromInput(input, Helper::pose);
	pthread_mutex_unlock(&pose_mutex);

	posePointCloudClient.call(posePointCloud);
	pcl::fromROSMsg(posePointCloud.response.pointCloud, pointCloud);
	return pointCloud;
}

vector<double> Helper::getQuatOrientation(geometry_msgs::Quaternion quat)
{
	double roll, pitch, yaw;
	tf::Quaternion q;
	tf::quaternionMsgToTF(quat, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	return {roll, pitch, yaw};
}

geometry_msgs::PoseStamped Helper::getPoseFromInput(vector<float> input, geometry_msgs::PoseWithCovarianceStamped pose)
{
	geometry_msgs::PoseStamped p_out;
	geometry_msgs::Pose currentPose, p, newPose;
	tf::Quaternion currentQuat;
	tf::Pose currentTfPose, newTfPose;

	currentPose = pose.pose.pose;
	
	p.position.z = input[3];
	p.position.x = -input[4]*input[12];
	p.position.y = 0.0;
	p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -input[12]*atan(input[5]),0.0);
	tf::quaternionMsgToTF(currentPose.orientation, currentQuat);
	tf::Transform currentTF(tf::Matrix3x3(currentQuat), tf::Vector3(currentPose.position.x,currentPose.position.y,currentPose.position.z));
	
	tf::poseMsgToTF(p, currentTfPose);
	newTfPose = currentTF * currentTfPose;
	tf::poseTFToMsg(newTfPose, newPose);
	
	p_out.header.stamp = ros::Time::now();
	p_out.header.frame_id = "/world";
	p_out.pose = newPose;
	return p_out;
}


vector<pcl::PointXYZ> Helper::pointCloudIntersection(pcl::PointCloud<pcl::PointXYZ> pointCloudA, pcl::PointCloud<pcl::PointXYZ> pointCloudB)
{
	vector<pcl::PointXYZ> commonPoints(pointCloudB.width * pointCloudB.height + pointCloudA.width * pointCloudA.height);
	vector<pcl::PointXYZ>::iterator it;
	it=set_intersection(pointCloudB.points.begin(), pointCloudB.points.end(), 
						pointCloudA.points.begin(), pointCloudA.points.end(), commonPoints.begin(),pointEqComparer());
	commonPoints.resize(it-commonPoints.begin());
	return commonPoints;
}

bool Helper::inLimits(float x, float y)
{
	if(MAP==1)
		return x>0.4 and y > 0.4 and x < 7.6 and y < 7.6 and (x<3.6 or x>4.4 or (x>=3.6 and x<=4.4 and y>6.4)); // map 1
	if(MAP==2)
		return x>0.4 and y > 0.4 and x < 7.6 and y < 7.6 and (x<2.6 or y>5.4); // map 2
	if(MAP==3)
		return x>0.4 and y > 0.4 and x < 7.6 and y < 7.6 and not ((x<2.9 or x>5.1) and (y>3.1 and y<4.9)); // map 3
	if(MAP==4)
		return x>-3.6 and y > -3.6 and x < 4.1 and y < 4.1 and (x<-1.9 or x>-1.1 or (x>=-1.9 and x<=-1.1 and y>1.9)) and (x<1.1 or x>1.9 or (x>=1.1 and x<=1.9 and y<-1.9)) and not(x<-3 and y>3); // map 4
	if(MAP==5)
		return x>1.2 and y > 0.4 and x < 10.5 and y < 7.6 and (x<3.6 or x>5.4 or (x>=3.6 and x<=5.4 and y>6.4)) and !(x > 7 and x<9 and y>3 and y<5); // rooms
	if(MAP==0)
		return x>=-6 and x<=0 and y>=-1 and y<=3; //training map
	return true;
}

bool Helper::collisionFree(float xi, float xf, float yi, float yf, float angle, int dir, double orientation)
{
	for(float i=0.05;i<=1;i+=0.05)
//	for(float x=xi;x!=xf;x+=dir*0.01*cos(orientation + angle))
//		for(float y=yi;y!=yf;y+=dir*0.01*sin(orientation + angle))
			if(!inLimits(xi+i*dir*0.01*cos(orientation + angle), yi+i*dir*0.01*cos(orientation + angle)))
				return false;
	return true;
}

vector<vector<float> > Helper::getTrajectories()
{
	float angle = PI/90.0, num_angles = NUM_ACTIONS, x, y;
	vector<vector<float> > inputs;
	vector<double> orientation = getQuatOrientation(robotWorldPose.orientation);
	
	for(float i=-num_angles*angle ; i<=num_angles*angle ; i+=angle)
	{	
		vector<float> inp = {0.0,0.0,0.0, 
								 cos(i), sin(i), tan(i),
								 0.0,0.0,0.0,0.0,0.0,0.0,
								 1.0,0.0,1.5};
		if(up)
		{	
			if(i<0 and !right)
				continue;
			if(i>0 and !left)
				continue;	
			x = robotWorldPose.position.x + cos(orientation[2] + i);
			y = robotWorldPose.position.y + sin(orientation[2] + i);
			if(inLimits(x,y))// and collisionFree(robotWorldPose.position.x, x, robotWorldPose.position.y, y, i, 1, orientation[2]))
				inputs.push_back(inp);
		}

		if(down)
		{
			if(i>0 and !left)
				continue;
			if(i<0 and !right)
				continue;	
			inp[3] *= -1.0;
			inp[4] *= -1.0;
			//inp[5] *= -1.0;
			inp[12] *= -1.0;
			x = robotWorldPose.position.x - cos(orientation[2] - i);
			y = robotWorldPose.position.y - sin(orientation[2] - i);
			if(inLimits(x,y))// and collisionFree(robotWorldPose.position.x, x, robotWorldPose.position.y, y, -i, -1, orientation[2]))
				inputs.push_back(inp);
		}
	}
	return inputs;
}

void Helper::saveFeatureExpectation(vector<vector<vector<float> > > episodeList, string fileName)
{
	ofstream feFile(fileName);
	for(auto episode : episodeList)
		for(auto rlStep : episode)
		{
			for(auto i : rlStep)
				feFile<< i << '\t';
			feFile << endl;
		}
	feFile.close();
}

void Helper::saveTempFeatureExpectation(vector<vector<float> > episode, string fileName)
{
	ofstream feFile(fileName, ios::app);
	for(auto rlStep : episode)
	{
		for(auto i : rlStep)
			feFile<< i << '\t';
		feFile << endl;
	}
	feFile.close();
}

vector<vector<vector<float> > > Helper::readFeatureExpectation(string fileName)
{
	vector<vector<vector<float> > > episodeList = vector<vector<vector<float> > >();
	vector<vector<float> > episode = vector<vector<float> >();
	ifstream infile(fileName);
	int num_episodes = 0;
	if(infile.good())
	{
		vector<float> step;
		for( string line; getline( infile, line ); )
		{
			stringstream ss;
		    ss.str(line);
			string item;
			while (getline(ss, item, '\t'))
				step.push_back(stof(item));
			episode.push_back(step);

			if(step.back()==1)
			{
				
				if(!episodeList.size() or (episodeList.size() and episode != episodeList.back()))
				{
					episodeList.push_back(episode);
					num_episodes++;
				}
				episode.clear();
			}
		}
	}

	return episodeList;
}