#ifndef SET_GOALS_H_
#define SET_GOALS_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>

#include<fstream>
#include<iostream>
#include<vector>
#include<sstream>
#include<string>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

///////////////////////////////////////////////////////////
class SetNavGoal
{
public:
    SetNavGoal();
	~SetNavGoal();
	void sendNavGoal(const double x, const double y, const double z, const double r);
	void cancelGoal();
	void cancelAllGoals();
	bool getNavResults();

	double curx,cury,curz,curr;
	bool setTaskEnable;
	int  navResult ;
	bool isFirstPose;

private:
	void doneCb(const actionlib::SimpleClientGoalState& state,
		        const move_base_msgs::MoveBaseResultConstPtr& result);
	void activeCb();
	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

	MoveBaseClient* setNavGoal_; 
    tf::StampedTransform world_pose;
	tf::TransformListener listener_;
	double last_x_set; double last_y_set;
};


/*-----------------------------------------------------------------*/
class GetGoals
{
public:
    GetGoals(const string& text_file_path_new);
	GetGoals();
	~GetGoals();
    bool readfile();
	void get_pose_array();

	vector<vector<double> > pose_array_6d;
	int goals_num;

private:
    ifstream inFile;
	string text_file_path;
	vector<vector<double> > pose_array_3d;
};

#endif



