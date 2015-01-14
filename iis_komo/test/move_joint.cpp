#include <ros/ros.h>
#include <iis_msgs/PlanTrajectory.h>
#include <iis_msgs/ExecuteTrajectory.h>
#include <string>

using namespace std;

/* plan and execute a trajectory for given arm and eef-link
 * to goal pose, defined by x,y,z,qX,qY,qZ,qW */
int main(int argc, char *argv[])
{
	if(argc < 9) {
		cerr << "Usage: move_joint ARM J0 J1 J2 J3 J4 J5 J6" << endl;
		return EXIT_FAILURE;
	}

	ros::init(argc, argv, "komo_move_joint");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	string arm = argv[1];

	/* -------------------- connect to planning and execution service -------------------------- */

	ROS_INFO("Creating service clients...");

	// create service clients for planning and execution...
	ros::ServiceClient plan_srv = nh.serviceClient<iis_msgs::PlanTrajectory>("motion_control/plan_trajectory");
	ros::ServiceClient exec_srv = nh.serviceClient<iis_msgs::ExecuteTrajectory>("motion_control/execute_trajectory");
	// ... and wait for the service to come up if necessary
	while(!plan_srv.exists()) {
		ROS_WARN("Waiting for planning service to come up...");
		plan_srv.waitForExistence(ros::Duration(10));
	}
	while(!exec_srv.exists()) {
		ROS_WARN("Waiting for execution service to come up...");
		exec_srv.waitForExistence(ros::Duration(10));
	}

	ROS_INFO("Service clients created!");

	/* ------------------- planning phase --------------------- */

	// define the service request
	iis_msgs::PlanTrajectoryRequest plan_request;
	// specify the planning group and the reference frame to plan for
	plan_request.planning_group = arm;
	// tell komo whether to tolerate collisions with the support surface or not
	plan_request.allow_support_surface_contact = false;
	// tell komo to use joint space planning mode
	plan_request.mode = plan_request.JOINT_SPACE_MODE;
	// specify target configuration
	plan_request.target.push_back(atof(argv[2])); // joint 0
	plan_request.target.push_back(atof(argv[3])); // joint 1
	plan_request.target.push_back(atof(argv[4])); // joint 2
	plan_request.target.push_back(atof(argv[5])); // joint 3
	plan_request.target.push_back(atof(argv[6])); // joint 4
	plan_request.target.push_back(atof(argv[7])); // joint 5
	plan_request.target.push_back(atof(argv[8])); // joint 6

	// create response object
	iis_msgs::PlanTrajectoryResponse plan_response;

	ROS_INFO("Planning trajectory...");

	// call the planning service and check result
	if(!plan_srv.call(plan_request, plan_response)) { // communication failure
		ROS_ERROR("Error on planning request!");
		return EXIT_FAILURE;
	}

	/* ------------------- validate result ------------------- */

	// check if planning was succesful
	if(plan_response.status == plan_response.FAILED) { // planning failure
		ROS_ERROR("Trajectory planning failed: %s", plan_response.error.c_str());
		return EXIT_FAILURE;
	}

	if(plan_response.status == plan_response.APPROXIMATE) { // only approximate solution
		ROS_WARN("Planner returned APPROXIMATE solution!");
		ROS_WARN("Status message: %s", plan_response.error.c_str());
	} else { // planning successful
		ROS_INFO("Planning request successful.");
	}

	ROS_INFO("Planning time  : %.3f", plan_response.planning_time);
	// the resulting configuration is the last point in the resulting trajectory
	size_t last = plan_response.trajectory.points.size()-1;
	vector<double> &state = plan_response.trajectory.points[last].positions;
	ROS_INFO("Resulting state: [ %f %f %f %f %f %f %f ]", state[0], state[1], state[2], state[3], state[4], state[5], state[6]);

	// ask user to confirm trajectory execution
	cout << "Confirm execution (y/n): ";
	string input;
	cin >> input;

	if(input != "y" && input != "Y") {
		ROS_INFO("Execution canceled!");
		return EXIT_SUCCESS;
	}

	/* ------------------- execution phase --------------------- */

	ROS_INFO("Executing trajectory...");

	iis_msgs::ExecuteTrajectoryRequest exec_request;
	exec_request.planning_group = arm;
	exec_request.trajectory = plan_response.trajectory;
	exec_request.velocity_factor = 0.9;

	iis_msgs::ExecuteTrajectoryResponse exec_response;

	if(!exec_srv.call(exec_request, exec_response)) {
		ROS_ERROR("Error on execution request!");
		return EXIT_FAILURE;
	}

	// check if execution was succesful
	if(!exec_response.result) { // execution failure
		ROS_ERROR("Trajectory execution failed!");
		return EXIT_FAILURE;
	}

	ROS_INFO("Position reached!");

	spinner.stop();

	return EXIT_SUCCESS;
}



