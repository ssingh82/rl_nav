#include <iostream>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <cmath>
#include <ros/package.h>
#include "SarsaLearner.h"

SarsaLearner::SarsaLearner()
{
	// Variables for reading values
	int stateDir;
	int stateHead;
	int stateFOV;
	//int statePFOV;
	float qValue;
	// Input file

	//Opening file and obtaining the data 
	ifstream qMatFile(ros::package::getPath("rl_nav")+"/qMatData.txt");
	ifstream wFile(ros::package::getPath("rl_nav")+"/wData.txt");

	// Check for failure
	if(!qMatFile)
	{
		cout<<"Invalid Q file"<<endl;
		qValid = false;
	}
	else 
	{
		qValid = true;
		for(int i = 0; i<STATE_DIR_MAX; i++)
			for(int j = 0; j<STATE_HEAD_MAX; j++)
				for(int k = 0; k<STATE_FOV_MAX; k++)
					{
						qMatFile >> stateDir >> stateHead >> stateFOV >> qValue;
						qMatrix[stateDir][stateHead][stateFOV] = qValue;
					}
	}

	
	if(!wFile)
	{
		cout<<"Invalid w file"<<endl;
		w = vector<float>(NUM_FEATURES_SA,0);
	}
	else 
	{
		w = vector<float>(NUM_FEATURES_SA);
		for(int i = 0; i<NUM_FEATURES_SA; i++)
			wFile >> w[i];
	}

	// Close
	qMatFile.close();
	wFile.close();

}

SarsaLearner::~SarsaLearner()
{
	// Output file
	ofstream qMatFile(ros::package::getPath("rl_nav")+"/qMatData.txt");
	ofstream wFile(ros::package::getPath("rl_nav")+"/wData.txt");

	float qValue;

	// Check for failure
	if(qMatFile)
	{
		cout<<"Writing to File"<<endl;
		for(int stateDir = 0; stateDir<STATE_DIR_MAX; stateDir++)
			for(int stateHead = 0; stateHead<STATE_HEAD_MAX; stateHead++)
				for(int stateFOV = 0; stateFOV<STATE_FOV_MAX; stateFOV++)
						qMatFile << stateDir << " " << stateHead << " " << stateFOV << " " << qMatrix[stateDir][stateHead][stateFOV] << endl;
	}

	if(wFile)
	{
		for(int i = 0; i<NUM_FEATURES_SA; i++)
			wFile << w[i] << " ";
		    wFile << endl;
	}

	// Close
	qMatFile.close();
	wFile.close();
}

// Function to return Q value
float SarsaLearner::getQ(vector<int> stateAction)
{
	if(!qValid)
		return 0.0;

	int stateDir = stateAction[0];
	int stateHead = stateAction[1];
	int stateFOV = stateAction[2];

	// Get q value
	return qMatrix[stateDir][stateHead][stateFOV];
}

/*float SarsaLearner::getReward(vector<int> stateAction)
{
	return -9.5 + stateAction[2]/2.0 - stateAction[1]/6.0 - 10*stateAction[3];
}
*/
float SarsaLearner::getReward(vector<int> stateAction)
{
	vector<float> phi;
	if(stateAction[0])
	{
		phi.push_back(0);
		phi.push_back(1);
	}
	else
	{
		phi.push_back(1);
		phi.push_back(0);
	}	
	
	phi.push_back(stateAction[1]/STATE_HEAD_MAX);
	phi.push_back(stateAction[2]/STATE_FOV_MAX);
	phi.push_back(1 - stateAction[3]/2.0);
	return inner_product(phi.begin(), phi.end(), w.begin(), 0.0);
}

// Function to update Q value
void SarsaLearner::updateQ(vector<int> stateAction, vector<int> nextStateAction)
{
	if(!qValid)
		return;

	int nextStateDir = nextStateAction[0];
	int nextStateHead = nextStateAction[1];
	int nextStateFOV = nextStateAction[2];
	
	// Get q value
	float qNext = qMatrix[nextStateDir][nextStateHead][nextStateFOV];

	updateQ(stateAction, qNext);
}

void SarsaLearner::updateQ(vector<int> stateAction, float qNext)
{
	if(!qValid)
		return;

	int stateDir = stateAction[0];
	int stateHead = stateAction[1];
	int stateFOV = stateAction[2];

	float Q = qMatrix[stateDir][stateHead][stateFOV];

	// SARSA update
	qMatrix[stateDir][stateHead][stateFOV] += SARSA_ALPHA * (getReward(stateAction) + SARSA_GAMMA * qNext - Q);
}

void SarsaLearner::episodeUpdate(vector<vector<vector<int> > > episodeList)
{
	for(vector<vector<vector<int> > >::iterator episode = episodeList.begin(); episode!=episodeList.end(); ++episode)
	{
		int i=0;
		for(vector<vector<int> >::iterator rlStep = episode->begin(); rlStep!=episode->end()-1; ++rlStep)
			updateQ(*rlStep, *next(rlStep));
		updateQ(episode->back(), 0);	
	}
	cout<<endl;
}