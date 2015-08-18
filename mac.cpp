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
#include <actionlib/client/simple_action_client.h>
using namespace std;

//Both the example code and this were made in rosbuild, so that's a thing.

//Hold onto your seats, this here is code to make the pr2 do the macarena.

//Currently working on moving the arms to the right positions.
//Next job is making the robot spin, and then loop the motions.

//Also the casters seem to be struggling to turn the robot for some reason.
//Weird.

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

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

	public:
		//! ROS node initialization
		RobotDriver(ros::NodeHandle &nh){
		  nh_ = nh;
		  //set up the publisher for the cmd_vel topic
		  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);

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
		}

		~RobotDriver(){
			delete traj_client_;
			delete traj_client_L;
		}

		pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory(){
			//our goal variable
		  pr2_controllers_msgs::JointTrajectoryGoal goal;

		  // First, the joint names, which apply to all waypoints
		  goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
		  goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
		  goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
		  goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
		  goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
		  goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
		  goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

		  // We will have four waypoints in this goal trajectory
		  goal.trajectory.points.resize(9);

		  // First trajectory point
		  // Positions
		  int ind = 0;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 0.0;
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = 0.0;
		  goal.trajectory.points[ind].positions[3] = 0.0;
		  goal.trajectory.points[ind].positions[4] = 3.0;
		  goal.trajectory.points[ind].positions[5] = 0.0;
		  goal.trajectory.points[ind].positions[6] = 0.0;

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 0.0; 
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = 0.0; 
		  goal.trajectory.points[ind].positions[3] = 0.0;
		  goal.trajectory.points[ind].positions[4] = 0.0; 
		  goal.trajectory.points[ind].positions[5] = -0.5;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 0.0; 
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = 0.0; 
		  goal.trajectory.points[ind].positions[3] = 0.0;
		  goal.trajectory.points[ind].positions[4] = 0.0; 
		  goal.trajectory.points[ind].positions[5] = -0.5;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(5.0);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 0.0; 
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = 0.0; 
		  goal.trajectory.points[ind].positions[3] = -1.5;
		  goal.trajectory.points[ind].positions[4] = 0.0; 
		  goal.trajectory.points[ind].positions[5] = 0.0;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(6.0);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 0.0; 
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = -1.0; 
		  goal.trajectory.points[ind].positions[3] = -1.5;
		  goal.trajectory.points[ind].positions[4] = -1.5; 
		  goal.trajectory.points[ind].positions[5] = -0.5;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(7.0);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 0.0; 
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = -1.0; 
		  goal.trajectory.points[ind].positions[3] = -1.5;
		  goal.trajectory.points[ind].positions[4] = -1.5; 
		  goal.trajectory.points[ind].positions[5] = -0.5;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(9.0);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = -1.0; 
		  goal.trajectory.points[ind].positions[1] = -1.0;
		  goal.trajectory.points[ind].positions[2] = 0.0; 
		  goal.trajectory.points[ind].positions[3] = -2.0;
		  goal.trajectory.points[ind].positions[4] = -1.5; 
		  goal.trajectory.points[ind].positions[5] = 0.0;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(11.0);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = -1.0; 
		  goal.trajectory.points[ind].positions[1] = -1.0;
		  goal.trajectory.points[ind].positions[2] = 0.0; 
		  goal.trajectory.points[ind].positions[3] = -2.0;
		  goal.trajectory.points[ind].positions[4] = -1.5; 
		  goal.trajectory.points[ind].positions[5] = 0.0;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(13.0);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = -1.0; 
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = -3.0; 
		  goal.trajectory.points[ind].positions[3] = -3.0;
		  goal.trajectory.points[ind].positions[4] = -1.5; 
		  goal.trajectory.points[ind].positions[5] = 0.0;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(15.0);


		  //we are done; return the goal
		  return goal;
 	 }

		pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectoryL(){
			//our goal variable
		  pr2_controllers_msgs::JointTrajectoryGoal goal;

		  // First, the joint names, which apply to all waypoints
		  goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
		  goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
		  goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
		  goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
		  goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
		  goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
		  goal.trajectory.joint_names.push_back("l_wrist_roll_joint");

		  // We will have four waypoints in this goal trajectory
		  goal.trajectory.points.resize(9);

		  // First trajectory point
		  // Positions
		  int ind = 0;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 0.0;
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = 0.0;
		  goal.trajectory.points[ind].positions[3] = 0.0;
		  goal.trajectory.points[ind].positions[4] = 3.0;
		  goal.trajectory.points[ind].positions[5] = 0.0;
		  goal.trajectory.points[ind].positions[6] = 0.0;

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(3.5);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 0.0;
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = 0.0;
		  goal.trajectory.points[ind].positions[3] = 0.0;
		  goal.trajectory.points[ind].positions[4] = 3.0;
		  goal.trajectory.points[ind].positions[5] = 0.0;
		  goal.trajectory.points[ind].positions[6] = 0.0;

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(4.5);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 0.0; 
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = 0.0; 
		  goal.trajectory.points[ind].positions[3] = 0.0;
		  goal.trajectory.points[ind].positions[4] = 0.0; 
		  goal.trajectory.points[ind].positions[5] = -0.5;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(5.5);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 0.0; 
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = 0.0; 
		  goal.trajectory.points[ind].positions[3] = 0.0;
		  goal.trajectory.points[ind].positions[4] = 0.0; 
		  goal.trajectory.points[ind].positions[5] = -0.5;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(7.5);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 0.0; 
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = 1.5; 
		  goal.trajectory.points[ind].positions[3] = -1.0;
		  goal.trajectory.points[ind].positions[4] = 0.0; 
		  goal.trajectory.points[ind].positions[5] = -0.5;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(9.5);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 0.0; 
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = 1.5; 
		  goal.trajectory.points[ind].positions[3] = -1.0;
		  goal.trajectory.points[ind].positions[4] = 0.0; 
		  goal.trajectory.points[ind].positions[5] = -0.5;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(11.5);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 1.0; 
		  goal.trajectory.points[ind].positions[1] = -1.0;
		  goal.trajectory.points[ind].positions[2] = 0.0; 
		  goal.trajectory.points[ind].positions[3] = -2.0;
		  goal.trajectory.points[ind].positions[4] = -1.5; 
		  goal.trajectory.points[ind].positions[5] = 0.0;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(13.5);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 1.0; 
		  goal.trajectory.points[ind].positions[1] = -1.0;
		  goal.trajectory.points[ind].positions[2] = 0.0; 
		  goal.trajectory.points[ind].positions[3] = -2.0;
		  goal.trajectory.points[ind].positions[4] = -1.5; 
		  goal.trajectory.points[ind].positions[5] = 0.0;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(15.5);

		  // Positions
		  ind += 1;
		  goal.trajectory.points[ind].positions.resize(7);
		  goal.trajectory.points[ind].positions[0] = 1.0; 
		  goal.trajectory.points[ind].positions[1] = 0.0;
		  goal.trajectory.points[ind].positions[2] = 3.0; 
		  goal.trajectory.points[ind].positions[3] = -3.0;
		  goal.trajectory.points[ind].positions[4] = 1.5; 
		  goal.trajectory.points[ind].positions[5] = 0.0;
		  goal.trajectory.points[ind].positions[6] = 0.0; 

		  // Velocities
		  goal.trajectory.points[ind].velocities.resize(7);
		  for (size_t j = 0; j < 7; ++j){
		    goal.trajectory.points[ind].velocities[j] = 0.0;
		  }
		  // To be reached 1.5 seconds after starting along the trajectory
		  goal.trajectory.points[ind].time_from_start = ros::Duration(17.5);

		  //we are done; return the goal
		  return goal;
 	 }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState(){
    return traj_client_->getState();
  }

  actionlib::SimpleClientGoalState getStateL(){
    return traj_client_L->getState();
  }

  //! Loop forever while sending commands
  bool moveit(pr2_controllers_msgs::JointTrajectoryGoal goal, pr2_controllers_msgs::JointTrajectoryGoal goalL){
			//we will be sending commands of type "twist" for the base
			geometry_msgs::Twist base_cmd;

			cout << "Test Running. Hit Ctrl+C to End. " << endl;
			ros::Duration(3.0).sleep(); // sleep for 3 seconds

			//Initial values for base
			base_cmd.linear.x = base_cmd.linear.y = 0;
			base_cmd.angular.z = 10.0;
			cmd_vel_pub_.publish(base_cmd);

			while(nh_.ok()){
		
				//Start the arm trajectory 1 second from now
				goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
				traj_client_->sendGoal(goal);
				traj_client_L->sendGoal(goalL);
				cout << "Sent Goal Trajectory" << endl;
				ros::Duration(18.0).sleep(); // sleep

				//base_cmd.linear.x = 1.0;
				base_cmd.angular.z = 3.0;
				for(int i = 0; i < 50; i++){	
					cmd_vel_pub_.publish(base_cmd);
					ros::Duration(0.1).sleep(); // sleep
				}
				//ros::Duration(1.0).sleep(); // sleep
				/*
				double nextTime = clock() + 5000;
				while(clock() < 5000){			
					double c = clock();
					//Divided by 10000 to try to exaggerate the slowing motion
					base_cmd.linear.y = -1*abs(sin(c/10000));
					cmd_vel_pub_.publish(base_cmd);
				}
				*/
				base_cmd.linear.x = 0;
				cmd_vel_pub_.publish(base_cmd);
			}
    return true;
  }
};

int main(int argc, char** argv){
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  driver.moveit(driver.armExtensionTrajectory(), driver.armExtensionTrajectoryL());

	return 0;
}
