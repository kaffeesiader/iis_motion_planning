#include <ros/ros.h>
#include <iis_msgs/PlanTrajectory.h>
#include <iis_msgs/ExecuteTrajectory.h>
#include <string>

using namespace std;

/* plan and execute a trajectory for given arm and eef-link
 * to goal pose, defined by x,y,z,qX,qY,qZ,qW */
int main(int argc, char *argv[])
{
	if(argc < 6) {
		cerr << "Usage: move_cart ARM EEF X Y Z [OPTIONAL rotX rotY rotZ rotW]" << endl;
		return EXIT_FAILURE;
	}

	ros::init(argc, argv, "komo_move");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	string arm = argv[1], eef = argv[2];

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
	plan_request.eef_link = eef;
	// tell komo whether to tolerate collisions with the support surface or not
	plan_request.allow_support_surface_contact = false;
	// tell komo to use cartesian space planning mode
	plan_request.mode = plan_request.CART_SPACE_MODE;
	// allow some tolerances for the goal position
	plan_request.position_tolerance.x = 0.01;
	plan_request.position_tolerance.y = 0.01;
	plan_request.position_tolerance.z = 0.01;

	plan_request.angular_tolerance.x = 0.05; // around 2.8 deg
	plan_request.angular_tolerance.y = 0.05; // around 2.8 deg
	plan_request.angular_tolerance.z = 0.05; // around 2.8 deg

	// specify target position
	plan_request.target.push_back(atof(argv[3])); // pos X
	plan_request.target.push_back(atof(argv[4])); // pos Y
	plan_request.target.push_back(atof(argv[5])); // pos Z

	// specify target orientation if provided
	if(argc >= 9) {
		plan_request.target.push_back(atof(argv[6])); // quat X or ROLL
		plan_request.target.push_back(atof(argv[7])); // quat Y or PITCH
		plan_request.target.push_back(atof(argv[8])); // quat Z or YAW

		// tell komo to align all axes of the end effector reference frame with the goal reference frame
		plan_request.axes_to_align = plan_request.X_AXIS | plan_request.Y_AXIS | plan_request.Z_AXIS;
	}

	if(argc == 10) {
		plan_request.target.push_back(atof(argv[9])); // quat W
	}
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
	ROS_INFO("Resulting pos : [%.5f,%.5f,%.5f]", p.position.x, p.position.y, p.position.z);
	ROS_INFO("Resulting quat: [%.5f,%.5f,%.5f,%.5f]", p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
	geometry_msgs::Vector3 &vec = plan_response.linear_error;
	ROS_INFO("Linear error : [%.5f,%.5f,%.5f]", vec.x, vec.y, vec.z);
	vec = plan_response.angular_error;
	ROS_INFO("Angular error: [%.5f,%.5f,%.5f]", vec.x, vec.y, vec.z);

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



