#include <set_nav_goals/set_goals.h>
#define PI 3.1415926

//----------------------------------------MyNode----------------------------------------//
SetNavGoal::SetNavGoal()
{
    setNavGoal_ = new MoveBaseClient("move_base", true);     
    while (!setNavGoal_->waitForServer(ros::Duration(60.0))) {
        ROS_INFO("---Waiting for the move_base action server to come up---"); //后面的时间间隔ros::Duration(),设置了等待的时间,若超过该时间,则中断该节点
    }
    ROS_INFO("---Connected to move base server---");
	navResult = 0;
	setTaskEnable = false;
	isFirstPose = true;
	curx = 0;
	cury = 0;
	curz = 0;
	curr = 0;
}

SetNavGoal::~SetNavGoal(){
    if(setNavGoal_ != NULL)
    {
        delete setNavGoal_;
        setNavGoal_ = NULL;
    }
}


void SetNavGoal::sendNavGoal(const double x, const double y, const double z, const double r)
{
	move_base_msgs::MoveBaseGoal goal;
    if (x != curx || y != cury ||isFirstPose){
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	goal.target_pose.pose.position.z = z;

	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(r);  //四元数
	goal.target_pose.pose.orientation.x = quat.x;
	goal.target_pose.pose.orientation.y = quat.y;
	goal.target_pose.pose.orientation.z = quat.z;
	goal.target_pose.pose.orientation.w = quat.w;


	//ROS_INFO("---Sending Goal_%d!---",(ind+1));
    
	setNavGoal_->sendGoal(goal,
            boost::bind(&SetNavGoal::doneCb, this, _1, _2),
            boost::bind(&SetNavGoal::activeCb, this),
            boost::bind(&SetNavGoal::feedbackCb, this, _1));
    isFirstPose = false; 
    }

}

//回调函数,当服务端完成时返回state和result执行该函数
void SetNavGoal::doneCb(const actionlib::SimpleClientGoalState& state,
                    const move_base_msgs::MoveBaseResultConstPtr& result){
    //ROS_INFO("RESULT %s", state.toString().c_str());
    // boost::unique_lock<boost::mutex> lock(wp_mutex_);
	
    switch (state.state_) {
    case actionlib::SimpleClientGoalState::ABORTED:
    {
        ROS_INFO("---Goal State: ABORTED---");
    }
    break;
    case actionlib::SimpleClientGoalState::SUCCEEDED:
    {
        ROS_INFO("---Goal State: SUCCEEDED---");
        setTaskEnable = true; 
        navResult = 1;
    }
    break;
    default:
        break;
    }
}
// Called once when the goal becomes active
void SetNavGoal::activeCb()
{
	ROS_INFO("Goal Received");
}

// Called every time feedback is received for the goal
void SetNavGoal::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	//ROS_INFO("Got base_position of Feedback");
	curx = (feedback->base_position.pose.position.x);
	cury = (feedback->base_position.pose.position.y);
	curz = (feedback->base_position.pose.position.z);
	curr = tf::getYaw(feedback->base_position.pose.orientation);
	//printf("pos:%f   %f  \n",curx,cury);
}

 /**
   * \brief Cancel the goal that we are currently pursuing
   */
void SetNavGoal::cancelGoal(){
    setNavGoal_->cancelGoal();
    //success_ = 1;
}

void SetNavGoal::cancelAllGoals(){
    setNavGoal_->cancelGoal();
    //success_ = 0;
}

bool SetNavGoal::getNavResults()
{
	return navResult;
}


/*
//----------------------------------------GetGoals----------------------------------------//
GetGoals::GetGoals(const string& text_file_path_new)
{
   text_file_path = text_file_path_new;
}

GetGoals::GetGoals()
{
   text_file_path = "/home/bianjingyang/catkin_ws/src/set_nav_goals/cfg/goals.txt";
}

GetGoals::~GetGoals()
{
    inFile.close();
}

bool GetGoals::readfile()
{
    inFile.open(text_file_path.c_str(),std::ios::in);
	if (!inFile.is_open())
	{
		ROS_INFO("---Open Goals File Failed!---");
		return false;
	}
}

void GetGoals::get_pose_array()
{
    string str;
	int line_num = 0;
	  
	while (getline(inFile, str))
	{
		istringstream input(str);  
		vector<double> tmp_3;
		double pose_3d;
		line_num++;               
		if (line_num > 4) {     
			while (input >> pose_3d) {
				tmp_3.push_back(pose_3d);
			}
			pose_array_3d.push_back(tmp_3);
		}
	}
    
    goals_num = pose_array_3d.size();
    for (int i = 0; i < goals_num; i++)
	{
        vector<double> tmp_6;
	    geometry_msgs::Quaternion quat;
        double tmp_yaw;
		for (int j = 0; j < 3; j++)
		{
			if(j<2) tmp_6.push_back(pose_array_3d[i][j]);  //push  x,y
			else{
			   tmp_6.push_back(0);                   //push  z=0
               tmp_yaw = (pose_array_3d[i][2])*PI/180;
               //ROS_INFO("-----------%f",tmp_yaw);
               quat = tf::createQuaternionMsgFromYaw(tmp_yaw);  //四元数
			   tmp_6.push_back(quat.x);         
			   tmp_6.push_back(quat.y);
			   tmp_6.push_back(quat.z);
			   tmp_6.push_back(quat.w);
			}
		}
		pose_array_6d.push_back(tmp_6);
	}
}
*/
