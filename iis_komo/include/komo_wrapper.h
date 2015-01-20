#ifndef KOMO_WRAPPER_H
#define KOMO_WRAPPER_H

#include <vector>

#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>

#include <iis_robot.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#define RESULT_SUCCESS 1
#define RESULT_FAILED 0
#define RESULT_APPROXIMATE 2
#define JNT_DISABLED 1
#define JNT_ENABLED 0

using namespace std;


namespace iis_komo {

struct PlanningResult {
	string error_msg;
	int status;
//	IISRobot::Path path;
	trajectory_msgs::JointTrajectory path;
	ors::Vector pos_error;
	ors::Vector ang_error;
	ors::Transformation resulting_pose;
	double planning_time;
};

struct PlanningRequest {
	string eef_link;
	ors::Transformation goal;
	bool position_only;
	bool allow_surface_contact;
};

class KomoWrapper {

public:

	KomoWrapper(const char *config_name);
	~KomoWrapper();

	// set the robot configuration to given state
	void setState(const sensor_msgs::JointState &state);
	void setActiveJoints(MT::Array<const char*> joints);

	// tolerance values used for trajectory validation
	void setPositionTolerance(const ors::Vector tolerance) { _pos_tolerance = tolerance; }
	void setAngularTolerance(const ors::Vector tolerance) { _ang_tolerance = tolerance; }

	void setPositionPrecision(double value) { _positionPrecision = value; }
	void setZeroVelocityPrecision(double value) { _zeroVelocityPrecision = value; }
	void setJointLimitPrecision(double value) { _jointLimitPrecision = value; }
	void setJointStatePrecision(double value) { _jointStatePrecision = value; }
	void setJointLimitMargin(double value) { _jointLimitMargin = value; }
	void setCollisionPrecision(double value) { _collisionPrecision = value; }
	void setCollisionMargin(double value) { _collisionMargin = value; }
	void setAlignmentPrecision(double value) { _alignmentPrecision = value; }
	void setIterations(double value) { _maxIterations = value; }
	void addCollisionObject(const string &object_id, const ors::ShapeType type, const ors::Transformation &pose,
							const arr size, const arr color=ARR(1.0,0.0,0.0));
	void removeCollisionObject(const string &object_id);
	/**
	 * Plans for given Cartesian space goal and given end effektor link. Planning outcome is stored
	 * in provided PlanningResult structure
	 *
	 * @param eef_link
	 * The end effector reference frame that should be aligned with goal reference frame
	 * @param goal
	 * The Transformation of the goal reference frame
	 * @param axes_to_align
	 * Which axes of the end effector reference frame should be aligned with the
	 * goal reference frame. This value is bit-coded: 1=X-Axis, 2=Y-Axis, 4=Z-Axis.
	 * For example to align all 3 axes use X-Axis + Y-Axis + Z-Axis = 7
	 * @param result
	 * A PlanningResult structure that is used to store the planning result information
	 */
	void plan(const string &eef_link, ors::Transformation &goal, int axes_to_align, PlanningResult &result);
	/**
	 * @brief plan
	 * @param goal_state
	 * @param result
	 */
	void plan(const vector<double> &goal_state, PlanningResult &result);

	void display(bool block = false, const char *msg = "Ready...");
	void display(const trajectory_msgs::JointTrajectory &trajectory);
	void allowContact(const char* link, bool allow);

private:

    ors::KinematicWorld *_world;

	ors::Vector _pos_tolerance;
	ors::Vector _ang_tolerance;
	vector<ors::Joint *> _active_joints;

	double _positionPrecision;
	double _zeroVelocityPrecision;
	double _jointLimitPrecision;
	double _jointLimitMargin;
	double _jointStatePrecision;
	double _collisionPrecision;
	double _collisionMargin;
	double _alignmentPrecision;
	double _maxIterations;

	MT::String _support_surface_name;
	MT::String _world_link_name;

	void pathToTrajectory(trajectory_msgs::JointTrajectory &traj, const arr &path);
	void setJointPosition(const string &name, const double pos);
};

}

#endif // KOMO_WRAPPER_H
