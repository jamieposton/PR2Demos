#include <iostream>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
using namespace std;

//Both the example code and this were made in rosbuild, so that's a thing.

//Hold onto your seats, this here is code to make the pr2 do the macarena.

//The casters seem to be struggling to turn the robot for some reason.
//Weird.

//New goal before the e-fit tours: make the head move sometimes along with the dance
//Makes the pr2 look more intelligent apparently
//Although how intelligent can you even look while doing the macarena

//Fixed trajectories. Now down to like 250 lines instead of 550+ yeah!!
//Everything looks a lot prettier than the safety study code.
//I might change that to reflect this for once.
//(Prettier because arrays of things instead of a new variable for each path)

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class RobotDriver{
	private:
		//! The node handle we'll be using for the base
		ros::NodeHandle nh_;
		//! We will be publishing to the "/base_controller/command" topic
		ros::Publisher cmd_vel_pub_;

		//Action client for the joint trajectory action
		//used to trigger the arm movement action
		TrajClient* traj_client_;
		TrajClient* traj_client_L;

		PointHeadClient* point_head_client_;


	public:
		//! ROS node initialization
		RobotDriver(ros::NodeHandle &nh){
			nh_ = nh;
			//set up the publisher for the cmd_vel topic
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);

			//Initialize the client for the Action interface to the head controller
			point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

			// tell the action client that we want to spin a thread by default
			traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
			traj_client_L = new TrajClient("l_arm_controller/joint_trajectory_action", true);

			// wait for action server to come up
			while(!traj_client_->waitForServer(ros::Duration(5.0))){
				ROS_INFO("Waiting for the joint_trajectory_action server");
			}
			while(!traj_client_L->waitForServer(ros::Duration(5.0))){
				ROS_INFO("Waiting for the joint_trajectory_action server");
			}

			//wait for head controller action server to come up 
			while(!point_head_client_->waitForServer(ros::Duration(5.0))){
				ROS_INFO("Waiting for the point_head_action server to come up");
			}
		}

		~RobotDriver(){
			delete traj_client_;
			delete traj_client_L;
			delete point_head_client_;
		}

		//! Points the high-def camera frame at a point in a given frame  
		void lookAt(double x, double y, double z)
		{
			//the goal message we will be sending
			pr2_controllers_msgs::PointHeadGoal goal;

			//the target point, expressed in the requested frame
			geometry_msgs::PointStamped point;
			point.header.frame_id = "base_link";
			point.point.x = x; point.point.y = y; point.point.z = z;
			goal.target = point;

			//we are pointing the high-def camera frame 
			//(pointing_axis defaults to X-axis)
			goal.pointing_frame = "high_def_frame";

			//take at least 0.5 seconds to get there
			goal.min_duration = ros::Duration(0.75);

			//and go no faster than 1 rad/s
			goal.max_velocity = 0.75;

			//send the goal
			point_head_client_->sendGoal(goal);

			//wait for it to get there (abort after 2 secs to prevent getting stuck)
			point_head_client_->waitForResult(ros::Duration(1));
		}

	void MoveArm(float* pos, float time){

		pr2_controllers_msgs::JointTrajectoryGoal goal;

		goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);

		// First, the joint names, which apply to all waypoints
		goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
		goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
		goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
		goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
		goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
		goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
		goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

		// We will have three waypoints in this goal trajectory
		goal.trajectory.points.resize(1);

		// Positions
		goal.trajectory.points[0].positions.resize(7);
		for(int i = 0; i < 7; i++){
			goal.trajectory.points[0].positions[i] = pos[i]; 
		}

		// Velocities
		goal.trajectory.points[0].velocities.resize(7);
		for (size_t j = 0; j < 7; ++j){
			goal.trajectory.points[0].velocities[j] = 0.0;
		}
		// To be reached 1.5 seconds after starting along the trajectory
		goal.trajectory.points[0].time_from_start = ros::Duration(time);

		traj_client_->sendGoal(goal);

		//wait for it to get there (abort after 2 secs to prevent getting stuck)
		traj_client_->waitForResult(ros::Duration(time+1.0));
	}

	void MoveArmL(float* pos, float time){

		pr2_controllers_msgs::JointTrajectoryGoal goal;

		goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);

		// First, the joint names, which apply to all waypoints
		goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
		goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
		goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
		goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
		goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
		goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
		goal.trajectory.joint_names.push_back("l_wrist_roll_joint");

		// We will have three waypoints in this goal trajectory
		goal.trajectory.points.resize(1);

		// Positions
		goal.trajectory.points[0].positions.resize(7);
		for(int i = 0; i < 7; i++){
			goal.trajectory.points[0].positions[i] = pos[i]; 
		}

		// Velocities
		goal.trajectory.points[0].velocities.resize(7);
		for (size_t j = 0; j < 7; ++j){
			goal.trajectory.points[0].velocities[j] = 0.0;
		}
		goal.trajectory.points[0].time_from_start = ros::Duration(time);
		traj_client_L->sendGoal(goal);

		traj_client_L->waitForResult(ros::Duration(time+1.0));
	}

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState(){
    return traj_client_->getState();
  }

  actionlib::SimpleClientGoalState getStateL(){
    return traj_client_L->getState();
  }

  //! Loop forever while sending commands
  bool moveit(int* choice){
			//we will be sending commands of type "twist" for the base
			geometry_msgs::Twist base_cmd;

			cout << "Test Running. Hit Ctrl+C to End. " << endl;

			if(choice[0] == 1){
				//Initial values for base
				base_cmd.linear.x = base_cmd.linear.y = 0;
				base_cmd.angular.z = 0.0;
				cmd_vel_pub_.publish(base_cmd);
			}

			if(choice[2] == 1){
				lookAt(0.0, -10.0, 1.0);
				ros::Duration(1.0).sleep(); // sleep
				lookAt(0.0, -10.0, -5.0);
				ros::Duration(1.0).sleep(); // sleep			
				lookAt(0.0, -10.0, 1.0);
				ros::Duration(1.0).sleep(); // sleep
				lookAt(10000.0, 0.0, 0.0);
				ros::Duration(1.0).sleep(); // sleep
			}
			cout << "Let's Dance!" << endl;

			while(nh_.ok() && (choice[1] == 1 || choice[0] == 1)){
		
				if(choice[1] == 1){

					float path[6][7] = {
						{0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0},
						{0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0},
						{0.0, 0.0, 0.0, -1.5, 0.0, -0.5, 0.0},
						{0.0, 0.0, -1.0, -1.5, -1.5, -0.5, 0.0},
						{-1.0, -1.0, 0.0, -2.0, -1.5, 0.0, 0.0},
						{-1.0, 0.0, -3.0, -3.0, -1.5, 0.0, 0.0}
					};
					float time[6] = {2.0, 1.0, 1.0, 1.0, 2.0, 2.0};

					float pathL[5][7] = {
						{0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0},
						{0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0},
						{0.0, 0.0, 1.5, -1.0, 0.0, -0.5, 0.0},
						{1.0, -1.0, 0.0, -2.0, -1.5, 0.0, 0.0},
						{1.0, 0.0, 3.0, -3.0, 1.5, 0.0, 0.0}
					};
					float timeL[5] = {2.0, 1.0, 2.0, 2.0, 2.0};

					for(int i = 0; i < 2; i++){
						MoveArm(path[i], time[i]);
						MoveArmL(pathL[i], timeL[i]);
					}
					MoveArm(path[2], time[2]);
					for(int i = 0; i < 3; i++){
						MoveArm(path[i+3], time[i+3]);
						MoveArmL(pathL[i+2], timeL[i+2]);
					}
				}

				if(choice[0] == 1){
					//base_cmd.linear.x = 1.0;
					base_cmd.angular.z = 1.0;
					for(int i = 0; i < 20; i++){	
						cmd_vel_pub_.publish(base_cmd);
						ros::Duration(0.1).sleep(); // sleep
					}
					base_cmd.linear.x = 0;
					cmd_vel_pub_.publish(base_cmd);
				}
				cout << "Ayyyyy Macarena!" << endl;
			}
    return true;
  }
};

void printHelp(){
	cout << "Possible options" << endl;
	cout << "b to run the base" << endl;
	cout << "a to run the arms" << endl;
	cout << "h to run the head" << endl;
	cout << "default is to sweep vertically" << endl;
}

int main(int argc, char** argv){

	int choice[3] = {0,0,0};
	//First is base
	//Second is arm
	//Third is head

	//Fourth used to be vertical or horizontal arm sweeps

	if(argc <= 1){
		cout << "Error: not enough arguments" << endl;
		printHelp();
		return 0;
	}

	for(int i = 1; i < argc; i++){
		switch(argv[i][0]){
			case 'b':
				choice[0] = 1;
				break;
			case 'a':
				choice[1] = 1;
				break;
			case 'h':
				choice[2] = 1;
				break;
			default:
				cout << "Incorrect argument given. Try again." << endl;
				printHelp();
				return 0;
		}
	}
	//init the ROS node
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;

	RobotDriver driver(nh);
	driver.moveit(choice);

	return 0;
}