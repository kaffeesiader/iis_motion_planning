#ifndef IO_HELPERS_H
#define IO_HELPERS_H

#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

using namespace std;

/**
 * @brief read_poses_from_file
 *
 * Reads poses from file with given name.
 * Expects a text file containing one pose per line in the format [x y z qx qy qz qw]
 *
 * @param fn
 * @param poses
 * @return
 */
bool read_poses_from_file(const char *fn, vector<geometry_msgs::Pose> &poses) {
	ROS_INFO("Opening input file '%s'", fn);

	ifstream input_file(fn);
	if(!input_file.is_open()) {
		ROS_ERROR("Unable to open input file '%s' for reading!", fn);
		return false;
	}

	string line;
	while(getline(input_file, line)) {
		geometry_msgs::Pose p;
		stringstream ss(line);

		ss >> p.position.x >> p.position.y >> p.position.z;
		ss >> p.orientation.x >> p.orientation.y >> p.orientation.z >> p.orientation.w;

		poses.push_back(p);
	}

	input_file.close();
	ROS_INFO("%d test poses loaded.", (int)poses.size());

	return true;
}

#endif // IO_HELPERS_H
