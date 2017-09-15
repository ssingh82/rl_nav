#include <vector>

using namespace std;

class SarsaLearner
{
protected:

	bool qValid;
	// Q Matrix dimension sizes
	static const int STATE_DIR_MAX = 2;
	static const int STATE_HEAD_MAX = 20;
	static const int STATE_FOV_MAX = 20;

	// SARSA parameters
	static constexpr float SARSA_ALPHA = 0.5f;
	static constexpr float SARSA_GAMMA = 0.9f;

	// Reward function features
	static const int NUM_FEATURES_SA = 5;


	float qMatrix[STATE_DIR_MAX][STATE_HEAD_MAX][STATE_FOV_MAX]; // Matrix for Q Values
	vector<float> w; //reward weights

public:
	SarsaLearner();
	~SarsaLearner();
	float getQ(vector<int> stateAction);
	void updateQ(vector<int> stateAction, vector<int> nextStateAction);
	void updateQ(vector<int> stateAction, float qNext);
	void episodeUpdate(vector<vector<vector<int> > > episodeList);
	float getReward(vector<int> stateAction);
};