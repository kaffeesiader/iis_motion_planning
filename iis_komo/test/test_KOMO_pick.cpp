#include <ros/ros.h>
#include <iis_msgs/PlanTrajectory.h>
#include <iis_msgs/ExecuteTrajectory.h>
#include <iis_schunk_hardware/GripCmd.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test_KOMO_pick");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	// used for controlling the hand
	ros::Publisher pub_grasp = nh.advertise<iis_schunk_hardware::GripCmd>("right_sdh/joint_control/grip_hand", 1, true);

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

	/* ------------------- reaching phase --------------------- */

	// define the service request
	iis_msgs::PlanTrajectoryRequest plan_request;
	// specify the planning group and the reference frame to plan for
	plan_request.planning_group = "right_arm";
	plan_request.eef_link = "right_sdh_tip_link";
	// allow some tolerances for the pregrasp position
	plan_request.position_tolerance.x = 0.03;
	plan_request.position_tolerance.y = 0.03;
	plan_request.position_tolerance.z = 0.03;

	plan_request.angular_tolerance.x = 0.1; // around 5 deg
	plan_request.angular_tolerance.y = 0.1; // around 5 deg
	plan_request.angular_tolerance.z = 0.1; // around 5 deg

	// specify target position
	plan_request.target.push_back(0.3); // pos x
	plan_request.target.push_back(0.3); // pos y
	plan_request.target.push_back(0.4); // pos z
	plan_request.target.push_back(0.0); // orient r
	plan_request.target.push_back(3.14);// orient p
	plan_request.target.push_back(0.0); // orient y

	// alternatively use quaternion ...
//	request.target.push_back(0.3); // pos x
//	request.target.push_back(0.3); // pos y
//	request.target.push_back(0.4); // pos z
//	request.target.push_back(0.0); // quat x
//	request.target.push_back(0.0); // quat y
//	request.target.push_back(0.0); // quat z
//	request.target.push_back(1.0); // quat w

	// ... or do position only planning
//	request.target.push_back(0.3); // pos x
//	request.target.push_back(0.3); // pos y
//	request.target.push_back(0.4); // pos z

	// create response object
	iis_msgs::PlanTrajectoryResponse plan_response;

	ROS_INFO("Planning and moving to pregrasp position");

	// call the planning service and check result
	if(!plan_srv.call(plan_request, plan_response)) { // communication failure
		ROS_ERROR("Error on planning request!");
		return EXIT_FAILURE;
	}

	// check if planning was succesful
	if(!plan_response.status) { // planning failure
		ROS_ERROR("Trajectory planning failed: %s", plan_response.error.c_str());
		return EXIT_FAILURE;
	}

	// execute reaching phase
	iis_msgs::ExecuteTrajectoryRequest exec_request;
	exec_request.planning_group = "right_arm";
	exec_request.trajectory = plan_response.trajectory;
	exec_request.velocity_factor = 0.5;

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

	ROS_INFO("Pregrasp position reached!");
	ROS_INFO("Preshaping hand");

	iis_schunk_hardware::GripCmd grasp;
	grasp.gripType = 1; // parallel
	grasp.closeRatio = 0.0;
	pub_grasp.publish(grasp);

	// allow the gripper to open
	sleep(2);

	/* ---------------------------- grasp phase ------------------------------- */

	ROS_INFO("Planning and moving to grasp position");

	plan_request.target.clear();
	plan_request.target.push_back(0.3); // pos x
	plan_request.target.push_back(0.3); // pos y
	plan_request.target.push_back(0.15); // pos z
	plan_request.target.push_back(0.0); // orient r
	plan_request.target.push_back(3.14);// orient p
	plan_request.target.push_back(0.0); // orient y

	// the tolerances should be stricter for grasp pose
	plan_request.position_tolerance.x = 0.003;
	plan_request.position_tolerance.y = 0.003;
	plan_request.position_tolerance.z = 0.003;

	plan_request.angular_tolerance.x = 0.1; // around 5 deg
	plan_request.angular_tolerance.y = 0.1; // around 5 deg
	plan_request.angular_tolerance.z = 0.1; // around 5 deg

	if(!plan_srv.call(plan_request, plan_response)) {
		ROS_ERROR("Error executing KOMO service request!");
		return EXIT_FAILURE;
	}

	// check if planning and execution was succesful
	if(!plan_response.status) {
		ROS_ERROR("Trajectory planning failed: %s", plan_response.error.c_str());
		return EXIT_FAILURE;
	}

	// execute grasp phase
	exec_request.planning_group = "right_arm";
	exec_request.trajectory = plan_response.trajectory;
	exec_request.velocity_factor = 0.5;

	if(!exec_srv.call(exec_request, exec_response)) {
		ROS_ERROR("Error on execution request!");
		return EXIT_FAILURE;
	}

	// check if execution was succesful
	if(!exec_response.result) { // execution failure
		ROS_ERROR("Trajectory execution failed!");
		return EXIT_FAILURE;
	}

	ROS_INFO("Grasp position reached!");
	ROS_INFO("Grasping object");

	grasp.closeRatio = 0.9;
	pub_grasp.publish(grasp);

	sleep(2);

	/* --------------------- depart phase --------------------------- */

	ROS_INFO("Planning and moving to postgrasp position");

	plan_request.target.clear();
	plan_request.target.push_back(0.0); // pos x
	plan_request.target.push_back(0.0); // pos y
	plan_request.target.push_back(0.4); // pos z
	plan_request.target.push_back(0.0); // orient r
	plan_request.target.push_back(3.14);// orient p
	plan_request.target.push_back(0.0); // orient y

	// the tolerances should be stricter for grasp pose
	plan_request.position_tolerance.x = 0.03;
	plan_request.position_tolerance.y = 0.03;
	plan_request.position_tolerance.z = 0.03;

	plan_request.angular_tolerance.x = 0.1; // around 5 deg
	plan_request.angular_tolerance.y = 0.1; // around 5 deg
	plan_request.angular_tolerance.z = 0.1; // around 5 deg

	if(!plan_srv.call(plan_request, plan_response)) {
		ROS_ERROR("Error executing KOMO service request!");
		return EXIT_FAILURE;
	}

	// check if planning and execution was succesful
	if(!plan_response.status) {
		ROS_ERROR("Trajectory planning failed: %s", plan_response.error.c_str());
		return EXIT_FAILURE;
	}

	// execute depart phase
	exec_request.planning_group = "right_arm";
	exec_request.trajectory = plan_response.trajectory;
	exec_request.velocity_factor = 0.5;

	if(!exec_srv.call(exec_request, exec_response)) {
		ROS_ERROR("Error on execution request!");
		return EXIT_FAILURE;
	}

	// check if execution was succesful
	if(!exec_response.result) { // execution failure
		ROS_ERROR("Trajectory execution failed!");
		return EXIT_FAILURE;
	}

	ROS_INFO("Postgrasp position reached!");
	ROS_INFO("Grasp completed!");

	spinner.stop();

	return EXIT_SUCCESS;
}

