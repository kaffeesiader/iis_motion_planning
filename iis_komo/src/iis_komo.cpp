#include <mutex>

#include <Ors/ors.h>
#include <ros/ros.h>

#include <iis_msgs/ExecuteTrajectory.h>
#include <iis_msgs/PlanTrajectory.h>
#include <iis_msgs/AddPrimitiveShape.h>
#include <std_msgs/String.h>

#include <robot_interface.h>
#include <komo_wrapper.h>
#include <time_parameterization.h>
#include <utils.h>

using namespace ros;
using namespace std;

namespace iis_komo {

class KomoInterface {

public:

	KomoInterface(NodeHandle &nh, KomoWrapper *komo, RobotInterface *robot)
	{
		_komo = komo;
		_robot = robot;
		_plan_srv = nh.advertiseService("motion_control/plan_trajectory", &KomoInterface::CBPlan, this);
		_execute_srv = nh.advertiseService("motion_control/execute_trajectory", &KomoInterface::CBExecute, this);
		_sub_add_primitive_shape = nh.subscribe("motion_control/scene/add_primitive_shape", 3, &KomoInterface::CBAddPrimitiveShape, this);
		_sub_remove_object = nh.subscribe("motion_control/scene/remove_object", 3, &KomoInterface::CBRemoveObject, this);
	}

	/* Starts a loop that frequently updates the UI with the current
	 * robot state */
	void run() {

		ros::Rate rate(0.5);

		while(ros::ok()) {
			// aquire the lock before updating the UI
			_mutex.lock();
			_komo->setState(_robot->getCurrentState());
			_komo->display();
			ROS_DEBUG("Update UI");
			_mutex.unlock();

			rate.sleep();
		}
	}

private:

	KomoWrapper *_komo;
	RobotInterface *_robot;
	TimeParameterization _time_param;

	ServiceServer _execute_srv;
	ServiceServer _plan_srv;
	Subscriber _sub_add_primitive_shape;
	Subscriber _sub_remove_object;
	// used to protecto the komo wrapper during planning
	Mutex _mutex;

	bool CBPlan(iis_msgs::PlanTrajectory::Request &request, iis_msgs::PlanTrajectory::Response &response) {

		ROS_INFO("Received planning request for link '%s' on planning group '%s'", request.eef_link.c_str(), request.planning_group.c_str());
		IISRobot::PlanninGroup group;

		if(request.planning_group == "left_arm") {
			group = IISRobot::LeftArm;
		} else if(request.planning_group == "right_arm") {
			group = IISRobot::RightArm;
		} else {
			string message = "Unknown planning group: " + request.planning_group;
			ROS_ERROR_STREAM(message);
			response.error = message;
			response.status = response.FAILED;
			return true;
		}

		// lock our komo instance for the duration of the planning request
		_mutex.lock();

		// merge the provided start state with current state...
		sensor_msgs::JointState state = _robot->getCurrentState();
		merge_states(state, request.start_state);
		// ... and use that state for planning
		_komo->setState(state);
		// tell komo which joints to use
		MT::Array<const char *> joint_names = IISRobot::get_jointnames_from_group(group);
		_komo->setActiveJoints(joint_names);
		// enable/disable collision checking with the support surface
		_komo->allowContact(MT::getParameter<MT::String>("KOMO/scene/supportSurfaceName"), request.allow_support_surface_contact);

		PlanningResult result;

		if(request.mode == request.CART_SPACE_MODE) {
			// determine the goal specification based on the size of the target vector
			size_t size = request.target.size();
			ors::Transformation goal;

			int axes_to_align = request.X_AXIS | request.Y_AXIS | request.Z_AXIS;

			switch (size) {
			case 3:
				// position only
				ROS_INFO("Specifying position only goal constraint.");

				goal.pos.x = request.target[0];
				goal.pos.y = request.target[1];
				goal.pos.z = request.target[2];

				axes_to_align = 0;

				break;
			case 6:
				// position and orientation based on roll, pitch, yaw
				ROS_INFO("Specifying position and orientation goal constraint (rpy).");

				goal.pos.x = request.target[0];
				goal.pos.y = request.target[1];
				goal.pos.z = request.target[2];

				goal.rot.setRpy(request.target[3], request.target[4], request.target[5]);

				if(request.axes_to_align == 0) {
					ROS_WARN("Not specified which axes to align - assuming all axes.");
				} else {
					axes_to_align = request.axes_to_align;
				}
				break;
			case 7:
				// position and orientation based on quaternion
				ROS_INFO("Specifying position and orientation goal constraint (quat).");

				goal.pos.x = request.target[0];
				goal.pos.y = request.target[1];
				goal.pos.z = request.target[2];

				goal.rot.set(request.target[6], request.target[3], request.target[4], request.target[5]);
				goal.rot.normalize();

				if(request.axes_to_align == 0) {
					ROS_WARN("Not specified which axes to align - assuming all axes.");
				} else {
					axes_to_align = request.axes_to_align;
				}
				break;
			default:
				string message = "Unable to plan - invalid target specification!";
				response.status = RESULT_FAILED;
				response.error = message;
				ROS_ERROR_STREAM(message);

				return true;
			}

			// define some default values to use if no tolerances provided...
			ors::Vector pos_tol = { 0.005, 0.005, 0.005 };
			ors::Vector ang_tol = { 0.1, 0.1, 0.1 };

			// set tolerance values if provided
			if((request.position_tolerance.x > 0.0) ||
				(request.position_tolerance.y > 0.0) ||
				(request.position_tolerance.z > 0.0))
			{
				pos_tol.x = request.position_tolerance.x;
				pos_tol.y = request.position_tolerance.y;
				pos_tol.z = request.position_tolerance.z;
			}

			if((request.angular_tolerance.x > 0.0) ||
				(request.angular_tolerance.y > 0.0) ||
				(request.angular_tolerance.z > 0.0))
			{
				ang_tol.x = request.angular_tolerance.x;
				ang_tol.y = request.angular_tolerance.y;
				ang_tol.z = request.angular_tolerance.z;
			}

			_komo->setPositionTolerance(pos_tol);
			_komo->setAngularTolerance(ang_tol);

			// compute the planning request
			_komo->plan(request.eef_link, goal, axes_to_align, result);

		} else if(request.mode == request.JOINT_SPACE_MODE) {
			// check for correct size of target vector
			if(request.target.size() != joint_names.N) {
				ROS_ERROR("Invalid size of target vector. It has to contain one value per joint");
				result.error_msg = "Invalid size of target vector. It has to contain one value per joint";
				result.status = RESULT_FAILED;
			} else {
				ROS_INFO("Planning for given joint space goal.");
				// use the values within request.target as goal state
				_komo->plan(request.target, result);
			}
		} else {
			ROS_ERROR("Unknown planning mode '%d'", request.mode);
			result.error_msg = "Unknown planning mode.";
			result.status = RESULT_FAILED;
		}

		/* ---------------- prepare the response ----------------------- */

		response.status = result.status;
		response.error = result.error_msg;
		response.planning_group = request.planning_group;

		if(result.status != RESULT_FAILED) {
			// this trajectory only contains joint positions, time-parameterization
			// is done on execution...
			response.trajectory = result.path;

			response.linear_error.x = result.pos_error.x;
			response.linear_error.y = result.pos_error.y;
			response.linear_error.z = result.pos_error.z;

			response.angular_error.x = result.ang_error.x;
			response.angular_error.y = result.ang_error.y;
			response.angular_error.z = result.ang_error.z;

			response.goal_pose.position.x = result.resulting_pose.pos.x;
			response.goal_pose.position.y = result.resulting_pose.pos.y;
			response.goal_pose.position.z = result.resulting_pose.pos.z;

			response.goal_pose.orientation.x = result.resulting_pose.rot.x;
			response.goal_pose.orientation.y = result.resulting_pose.rot.y;
			response.goal_pose.orientation.z = result.resulting_pose.rot.z;
			response.goal_pose.orientation.w = result.resulting_pose.rot.w;

			response.planning_time = result.planning_time;
			response.status = result.status;

			if(result.status == RESULT_APPROXIMATE) {
				ROS_WARN("Resulting solution is APPROXIMATE!");
				ROS_WARN("Error: %s", result.error_msg.c_str());
			}

			_komo->display(response.trajectory);
			sleep(1);

			ROS_INFO("Planning request completed!");
		} else {
			ROS_ERROR("Planning failed: %s", result.error_msg.c_str());
		}

		// end of critical section
		_mutex.unlock();

		return true;
	}

	bool CBExecute(iis_msgs::ExecuteTrajectory::Request &request, iis_msgs::ExecuteTrajectory::Response &response) {

		ROS_INFO("Received trajectory execution request for planning group '%s'", request.planning_group.c_str());
		IISRobot::PlanninGroup group;

		if(request.planning_group == "left_arm") {
			group = IISRobot::LeftArm;
		} else if(request.planning_group == "right_arm") {
			group = IISRobot::RightArm;
		} else {
			ROS_ERROR("Unknown planning group '%s'.", request.planning_group.c_str());
			response.result = false;
			return true;
		}

		if((request.velocity_factor <= 0.0)||(request.velocity_factor > 1.0)) {
			ROS_ERROR("Velocity factor has to be a value between 0 ant 1!");
			response.result = false;
			return true;
		}

		double vel_factor = request.velocity_factor;

		ROS_INFO("Computing time parameterization (velocity factor: %.2f)", vel_factor);
		_time_param.clearTimeParams(request.trajectory);
		_time_param.set_velocity_factor(vel_factor);
		_time_param.computeTimeStamps(request.trajectory);

//		for (int i = 0; i < request.trajectory.points.size(); ++i) {
//			trajectory_msgs::JointTrajectoryPoint &pt = request.trajectory.points[i];
//			cout << "TFS: " << pt.time_from_start << endl;
//		}

		ROS_INFO("Executing trajectory...");

		if(_robot->execute(group, request.trajectory)) {
			response.result = true;
		} else {
			response.result = false;
		}

		return true;
	}

	void CBAddPrimitiveShape(const iis_msgs::AddPrimitiveShapePtr &msg) {

		ROS_INFO("Adding object '%s' to planning scene", msg->object_id.c_str());

		ors::ShapeType type;
		arr size = zeros(4);

		switch(msg->type) {

		case iis_msgs::AddPrimitiveShape::BOX:
			type = ors::boxST;
			size(0) = msg->dimensions[iis_msgs::AddPrimitiveShape::BOX_X];
			size(1) = msg->dimensions[iis_msgs::AddPrimitiveShape::BOX_Y];
			size(2) = msg->dimensions[iis_msgs::AddPrimitiveShape::BOX_Z];
			break;
		case iis_msgs::AddPrimitiveShape::CYLINDER:
			type = ors::cylinderST;
			size(2) = msg->dimensions[iis_msgs::AddPrimitiveShape::CYLINDER_HEIGHT];
			// last component defines cylinder diameter
			size(3) = msg->dimensions[iis_msgs::AddPrimitiveShape::CYLINDER_RADIUS] * 2;
			break;
		case iis_msgs::AddPrimitiveShape::SPHERE:
			type = ors::sphereST;
			// last component defines sphere radius
			size(3) = msg->dimensions[iis_msgs::AddPrimitiveShape::SPHERE_RADIUS];
			break;
		default:
			ROS_ERROR("Failed to add shape - given shape type is not supported!");
			return;
		}

		ors::Transformation objTrans;
		poseToTransformation(msg->pose, objTrans);

		_komo->addCollisionObject(msg->object_id, type, objTrans, size);
	}

	void CBRemoveObject(const std_msgs::StringPtr &msg) {
		ROS_INFO("Removing object '%s' from scene", msg->data.c_str());
		_komo->removeCollisionObject(msg->data);
	}

};

}

int main(int argc, char *argv[])
{
	MT::initCmdLine(argc, argv);
	ros::init(argc, argv, "iis_komo");
	AsyncSpinner spinner(2);
	spinner.start();

	NodeHandle nh;

	ROS_INFO("Starting IIS_KOMO node in namespace '%s'...", nh.getNamespace().c_str());

	iis_komo::KomoWrapper wrapper(MT::getParameter<MT::String>("KOMO/robot_config"));
	iis_komo::RobotInterface robot(nh);
	iis_komo::KomoInterface ki(nh, &wrapper, &robot);

	ki.run();

	spinner.stop();
	ROS_INFO("IIS_KOMO node shutdown completed.");

	return EXIT_SUCCESS;
}

