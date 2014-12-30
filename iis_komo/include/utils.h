#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <iis_robot.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <Ors/ors.h>

using namespace std;
using namespace iis_komo;

int indexOf(const vector<string> &list, const string &value) {
	vector<string>::const_iterator it = find(list.begin(), list.end(), value);
	if(it == list.end())
		return -1;
	else
		return it - list.begin();
}

void merge_states(sensor_msgs::JointState &state, const sensor_msgs::JointState &other) {
	CHECK(other.name.size() == other.position.size(),"");
	vector<string> &names = state.name;

	for (size_t i = 0; i < other.name.size(); ++i) {
		int idx = indexOf(names, other.name[i]);
		if(idx == -1)
			cerr << "Unknown joint: " << other.name[i] << endl;
		else
			state.position[idx] = other.position[i];
	}
}

void komo_path_to_joint_traj(const IISRobot::PlanninGroup &group,
							 const IISRobot::Path &path,
							 trajectory_msgs::JointTrajectory &trajectory) {

	// set joint names
	MT::Array<const char *> joint_names = IISRobot::get_jointnames_from_group(group);
	// visualize joint names (testing...)
	for (int i = 0; i < 7; ++i) {
		trajectory.joint_names.push_back(joint_names(i));
//		cout << joint_names(i) << endl;
	}

	// create waypoints
	int num_points = path.d0;
	trajectory.points.resize(num_points);
	for (int i = 0; i < num_points; ++i) {
		const arr &wp = path[i];
		CHECK(wp.N==joint_names.N, "Invalid path for given group!");
		trajectory_msgs::JointTrajectoryPoint &point = trajectory.points[i];
		for (int j = 0; j < joint_names.N; ++j) {
			point.positions.push_back(wp(j));
		}
	}

}

void poseToTransformation(const geometry_msgs::Pose &pose, ors::Transformation &trans) {
	trans.pos.x = pose.position.x;
	trans.pos.y = pose.position.y;
	trans.pos.z = pose.position.z;

	trans.rot.x = pose.orientation.x;
	trans.rot.y = pose.orientation.y;
	trans.rot.z = pose.orientation.z;
	trans.rot.w = pose.orientation.w;
	trans.rot.normalize();
}

#endif // UTILS_H
