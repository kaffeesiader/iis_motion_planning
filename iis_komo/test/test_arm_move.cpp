#include <ros/ros.h>
#include <iis_msgs/PlanTrajectory.h>
#include <iis_msgs/ExecuteTrajectory.h>
#include <string>

using namespace std;

#define ARM "right"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test_KOMO_pick");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(1);
	spinner.start();

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

	string arm(ARM);

	// define the service request
	iis_msgs::PlanTrajectoryRequest plan_request;
	// specify the planning group and the reference frame to plan for
	plan_request.planning_group = arm + "_arm";
	plan_request.eef_link = arm + "_sdh_palm_link";
	// allow some tolerances for the pregrasp position
	plan_request.position_tolerance.x = 0.03;
	plan_request.position_tolerance.y = 0.03;
	plan_request.position_tolerance.z = 0.03;

	plan_request.angular_tolerance.x = 0.05; // around 5 deg
	plan_request.angular_tolerance.y = 0.05; // around 5 deg
	plan_request.angular_tolerance.z = 0.05; // around 5 deg

	// specify target position
//	plan_request.target.push_back(0.2); // pos x
//	plan_request.target.push_back(1.2); // pos y
//	plan_request.target.push_back(0.6); // pos z
//	plan_request.target.push_back(0.0); // orient r
//	plan_request.target.push_back(1.57);// orient p
//	plan_request.target.push_back(0.0); // orient y

	// alternatively use quaternion ...
	plan_request.target.push_back(0.3); // pos x
	plan_request.target.push_back(0.3); // pos y
	plan_request.target.push_back(0.6); // pos z
	plan_request.target.push_back(0.0); // quat x
	plan_request.target.push_back(0.707107); // quat y
	plan_request.target.push_back(0.0); // quat z
	plan_request.target.push_back(0.0); // quat w
//	(-8.75815e-06 -1 1.07242e-06 -2.34442e-06)

	// ... or do position only planning
//	request.target.push_back(0.3); // pos x
//	request.target.push_back(0.3); // pos y
//	request.target.push_back(0.4); // pos z

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

	ROS_INFO("Planning time: %.3f", plan_response.planning_time);
	geometry_msgs::Pose &p = plan_response.goal_pose;
	ROS_INFO("Resulting pos: [%.5f,%.5f,%.5f]", p.position.x, p.position.y, p.position.z);
	ROS_INFO("Resulting ori: [%.5f,%.5f,%.5f,%.5f]", p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
	geometry_msgs::Vector3 &vec = plan_response.linear_error;
	ROS_INFO("Linear error : [%.5f,%.5f,%.5f]", vec.x, vec.y, vec.z);
	vec = plan_response.angular_error;
	ROS_INFO("Angular error: [%.5f,%.5f,%.5f]", vec.x, vec.y, vec.z);

	/* ------------------- execution phase --------------------- */

	ROS_INFO("Executing trajectory...");

	// execute trajectory
	iis_msgs::ExecuteTrajectoryRequest exec_request;
	exec_request.planning_group = arm + "_arm";
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

	ROS_INFO("Position reached!");

	spinner.stop();

	return EXIT_SUCCESS;
}


