/*
 * test_gripper_action_client.cpp
 *
 *  Created on: Nov 9, 2013
 *      Author: martin
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace actionlib;
using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_gripper");

    string arm = "right";
    if(argc > 1) {
        arm = argv[1];
    }

    string topic = arm + "_sdh/follow_joint_trajectory/";
	// create the action client
	// true causes the client to spin its own thread
    SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac(topic, true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back(arm + "_sdh_finger_12_joint");
    goal.trajectory.joint_names.push_back(arm + "_sdh_finger_13_joint");
    goal.trajectory.joint_names.push_back(arm + "_sdh_finger_22_joint");
    goal.trajectory.joint_names.push_back(arm + "_sdh_finger_23_joint");
    goal.trajectory.joint_names.push_back(arm + "_sdh_knuckle_joint");
    goal.trajectory.joint_names.push_back(arm + "_sdh_thumb_2_joint");
    goal.trajectory.joint_names.push_back(arm + "_sdh_thumb_3_joint");

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(7);

    point.positions[0] = (-M_PI / 6);
    point.positions[1] = (M_PI / 9);
    point.positions[2] = (-M_PI / 6);
    point.positions[3] = (M_PI / 9);
    point.positions[4] = (M_PI / 4);
    point.positions[5] = (-M_PI / 6);
    point.positions[6] = (M_PI / 9);
    // allow 4 seconds until goal pose
    point.time_from_start = ros::Duration(2);
    // first goal
    goal.trajectory.points.push_back(point);

    ROS_INFO("Executing gripper action...");
    ac.sendGoal(goal);

    //wait for the action to return
    if (ac.waitForResult(ros::Duration(30.0))) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not finish before the time out.");
    }

    point.positions[0] = 0;
    point.positions[1] = 0;
    point.positions[2] = 0;
    point.positions[3] = 0;
    point.positions[4] = 0;
    point.positions[5] = 0;
    point.positions[6] = 0;

    point.time_from_start = ros::Duration(2);
    // second goal
    goal.trajectory.points.clear();
    goal.trajectory.points.push_back(point);

    ROS_INFO("Executing gripper action...");
    ac.sendGoal(goal);

    //wait for the action to return
    if (ac.waitForResult(ros::Duration(30.0))) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not finish before the time out.");
    }

    return EXIT_SUCCESS;
}

