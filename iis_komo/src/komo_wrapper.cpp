/**
 *
 *
 *
 *
 **/
#include <komo_wrapper.h>
//#include <utils.h>
#include <komo_helpers.h>
#include <ros/ros.h>
#include <boost/format.hpp>

using namespace std;

namespace iis_komo {


KomoWrapper::KomoWrapper(const char *config_name)
{
	cout << "Loading robot description from file '" << string(config_name) << "'." << endl;
	_world = new ors::KinematicWorld(config_name);

	// initialize list of activated joints
	for(ors::Joint *j:_world->joints) {
		if(j->agent == 0)
			_active_joints.push_back(j);
	}

	double def_pos_tolerance = MT::getParameter<double>("KOMO/moveTo/defaultPositionTolerance", 0.005);
	double def_ang_tolerance = MT::getParameter<double>("KOMO/moveTo/defaultAngularTolerance", 0.1);

	_pos_tolerance = {def_pos_tolerance, def_pos_tolerance, def_pos_tolerance};
	_ang_tolerance = {def_ang_tolerance, def_ang_tolerance, def_ang_tolerance};

	//-- parameters
	_positionPrecision = MT::getParameter<double>("KOMO/moveTo/positionPrecision", 1e4); // original 1e3
	_collisionPrecision = MT::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
	_collisionMargin = MT::getParameter<double>("KOMO/moveTo/collisionMargin", .1);
	_jointLimitPrecision = MT::getParameter<double>("KOMO/moveTo/jointLimitPrecision", 0.1);
	_jointLimitMargin = MT::getParameter<double>("KOMO/moveTo/jointLimitMargin", 1e5);
	_jointStatePrecision = MT::getParameter<double>("KOMO/moveTo/jointStatePrecision", 1e5);
	_zeroVelocityPrecision = MT::getParameter<double>("KOMO/moveTo/zeroVelocityPrecision", 1e1);
	_alignmentPrecision = MT::getParameter<double>("KOMO/moveTo/alignmentPrecision", 1e4); // original 1e3
	_maxIterations = MT::getParameter<double>("KOMO/moveTo/maxIterations", 1);

	_support_surface_name = MT::getParameter<MT::String>("KOMO/scene/supportSurfaceName");
	_world_link_name = MT::getParameter<MT::String>("KOMO/scene/worldLinkName");
}

KomoWrapper::~KomoWrapper() {
    if(_world) {
        delete _world;
	}
}

void KomoWrapper::setJointPosition(const string &name, const double pos)
{
	ors::Joint *jnt = _world->getJointByName(name.c_str());
	if(!jnt)
		cerr << "Unable to set joint position - no joint with name '" << name << "' found!" << endl;
	else
		jnt->Q.rot.setRad(pos, 1, 0, 0);
}

void KomoWrapper::setState(const sensor_msgs::JointState &state)
{
	CHECK(state.name.size() == state.position.size(), "Illegal joint states!");
	for (size_t i = 0; i < state.name.size(); ++i) {
		setJointPosition(state.name[i], state.position[i]);
	}
	// update mimic joints
	for(ors::Joint *j:_world->joints) {
		if(j->mimic)
			// not very nice, but should work...
			j->Q.rot = j->mimic->Q.rot;
	}
	_world->calc_fwdPropagateFrames();
	_world->calc_q_from_Q();
}

void KomoWrapper::setActiveJoints(MT::Array<const char *> joints)
{
	// TODO: currently it is not possible to dissable specific joints because of swift error
	//       maybe this issue can be solved in future

	// disable currently active joints first
//	for(ors::Joint *jnt : _active_joints) {
//		jnt->agent = JNT_DISABLED;
//		cout << "Joint " << jnt->name << " disabled" << endl;
//	}

	_active_joints.clear();

	// enable all joints in given list
	for(const char *joint_name : joints) {
		ors::Joint *jnt = _world->getJointByName(joint_name);
		if(jnt) {
//			jnt->agent = JNT_ENABLED;
			_active_joints.push_back(jnt);
//			cout << "Joint " << jnt->name << " enabled" << endl;
		}
		else
			cerr << "Joint with name '" << joint_name << "' does not exist!" << endl;
	}
	// what we do here is a hack, because working with agents is still buggy...
	// simply force komo to rebuild the qIndex
//	_world->qdim(0) = UINT_MAX;
//	_world->calc_q_from_Q();
//	cout << "New dimension: " << _world->getJointStateDimension() << endl;
//	cout << "Joints activated" << endl;
}

void KomoWrapper::addCollisionObject(const string &object_id, const ors::ShapeType type,
									 const ors::Transformation &pose, const arr size, const arr color)
{
	CHECK(size.N==4,"Invalid size!");
	CHECK(color.N==3,"Invalid color!");
	// check if given shape already exists...
	ors::Shape *shape = _world->getShapeByName(object_id.c_str());
	if(shape) {
		cerr << "Object with name '" << object_id << "'' already exists in current scene!" << endl;
		cerr << "It will be deleted!" << endl;
		delete shape;
	}

	ors::Body *body = _world->getBodyByName(_world_link_name);
	CHECK(body, "World link body not found within scene!");

	shape = new ors::Shape(*_world, *body, NULL, false);
	shape->name = STRING(object_id);
	shape->type = type;

	cout << "Size: " << size << endl;
	shape->size[0] = size(0);
	shape->size[1] = size(1);
	shape->size[2] = size(2);
	shape->size[3] = size(3);

	cout << "Color: " << color << endl;
	shape->color[0] = color(0);
	shape->color[1] = color(1);
	shape->color[2] = color(2);

	shape->rel = pose;

	// enable collision checking
	shape->cont = true;

	// somehow necessary, otherwise swift crashes...
	shape->parseAts();

	cout << "Pose: " << pose << endl;

	_world->calc_fwdPropagateFrames();
	_world->swift().initActivations(*_world);
}

void KomoWrapper::removeCollisionObject(const string &object_id)
{
	ors::Shape *shape = _world->getShapeByName(object_id.c_str());
	if(!shape) {
		cerr << "Object with name '" << object_id << "'' not found!" << endl;
	} else {
		delete shape;
	}
}

void KomoWrapper::plan(const string &eef_link, ors::Transformation &goal, int axes_to_align, PlanningResult &result)
{
	// ensure that some joints have been enabled, otherwise planning is not possible
	CHECK(_active_joints.size() > 0, "Unable to plan - at least 1 joint has to be activated!");
	MT::timerStart();

	ors::Shape *target = _world->getShapeByName("target");
	if(!target) {
		result.error_msg = "Unable to find shape with name 'target' within model.";
		cerr << result.error_msg << endl;
		result.status = RESULT_FAILED;
		return;
	}

	ors::Shape *endeff = _world->getShapeByName(eef_link.c_str());
	if(!endeff) {
		result.error_msg = "Unable to find link with name '" + eef_link + "' within model.";
		cerr << result.error_msg << endl;
		result.status = RESULT_FAILED;
		return;
	}

	// set target position...
	target->rel.pos = goal.pos;
	// ...and orientation
	target->rel.rot = goal.rot;

	_world->calc_fwdPropagateShapeFrames();
	display(false, "planning...");

	target->cont = false; // don't know if this is necessary...

	//-- set up the MotionProblem
	MotionProblem MP(*_world);
	MP.loadTransitionParameters();
	_world->swift().initActivations(*_world);

	TaskCost *c;
	// TaskMap for end effector position
	c = MP.addTask("EEF_position", new DefaultTaskMap(posTMT, endeff->index, NoVector, target->index, NoVector));
	c->setCostSpecs(MP.T, MP.T, {0.}, _positionPrecision);

	// TaskMap for zero velocity at goal
	c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, *_world));
	c->setCostSpecs(MP.T, MP.T, {0.}, _zeroVelocityPrecision);
	c->map.order = 1; //make this a velocity variable!

	// Add a small bias towards a neutral joint configuration (don't know if that helps...)
//	c = MP.addTask("state", new DefaultTaskMap(qItselfTMT, w));
//	c->setCostSpecs(0, MP.T, {0.}, 1e-3);

	// TaskMaps for eef alignment
	for(uint i=0;i<3;i++) if(axes_to_align & (1 << i)) {
		ors::Vector axis;
		axis.setZero();
		axis(i) = 1.;
		c = MP.addTask(STRING("allign_" << i), new DefaultTaskMap(vecAlignTMT, endeff->index, axis, target->index, axis));
		c->setCostSpecs(MP.T, MP.T, {1.}, _alignmentPrecision); // ARR(0.) -> make axis orthogonal!
	}

	// Constraint to enforce joint limits on all time slices
	LimitsConstraint *lc = new LimitsConstraint();
	lc->margin = 0.005;
	c = MP.addTask("Joint_limits", lc);
	c->setCostSpecs(0, MP.T, {0.}, _jointLimitPrecision);

	// enable collision checking
	TaskCost *colCost = MP.addTask("Collisions", new ProxyTaskMap(allPTMT, {0}, _collisionMargin));
	colCost->setCostSpecs(MP.T, MP.T, {0.}, _collisionPrecision);

	// TaskMap for transition costs
	c = MP.addTask("Transitions", new TransitionTaskMap(*_world));
//	c->map.order = 1; // penalize velocities
	c->map.order = 2; // penalize accelerations
	c->setCostSpecs(0, MP.T, {0.}, 1e0);

	//-- create the Optimization problem (of type kOrderMarkov)
	MotionProblemFunction MF(MP);
	//-- optimization process

	ors::KinematicWorld::setJointStateCount = 0;
	arr state, traj;
	ors::Vector posError, angError;

	_world->getJointState(state);

	for(int j = 0; j < _maxIterations; ++j) {

		cout << "Iteration " << j+1 << endl;

		MP.prefix.clear();
		MP.x0 = state;

		arr x = replicate (MP.x0, MP.T+1);
		rndGauss(x, .01, true); //don't initialize at a singular config

		optConstrained(x, NoArr, Convert(MF), OPT(verbose=0, stopIters=100, maxStep=.5, stepInc=2., allowOverstep=false));
		MP.costReport(false);

		// ensure that all joints are within calculated limits before do collision validation
		ensureJointLimits(*_world, x);

		if(!validateCollisions(*_world, x, result.error_msg)) {
			result.status = RESULT_FAILED;
			cerr << "Validation failed!" << endl;
			return;
		}
		// not necessary any more - just for test purposes...
		if(!validateJointLimits(*_world, x, result.error_msg)) {
			result.status = RESULT_FAILED;
			cerr << "Validation failed!" << endl;
			return;
		}

		traj.append(x);

		// set world to final state of resulting path and calculate the transformations
		state = x[x.d0-1];
		_world->setJointState(state);

		computePositionError(*endeff, *target, posError);
		computeAlignmentError(*endeff, *target, angError, axes_to_align);

		if(withinTolerance(posError, _pos_tolerance) &&
		   withinTolerance(angError, _ang_tolerance, axes_to_align))
		{
			break;
		}
	}

	CHECK(traj.d0 > 0, "Trajectory is empty...");

	result.planning_time = MT::timerRead();

	cout << "Optimization process finished" << endl;
	cout << "Optimization time:  " << result.planning_time << endl;
	cout << "SetJointStateCount: " << ors::KinematicWorld::setJointStateCount << endl;


	/* ---------------------- validation -------------------------*/

	// consider plan to be successful unless validation shows
	// something different...
	result.status = RESULT_SUCCESS;

	// so far, our path is valid but maybe the goal tolerances are violated...
	pathToTrajectory(result.path, traj);
	result.resulting_pose = endeff->X;
	result.pos_error = posError;
	result.ang_error = angError;

	cout << "EEF final pos:   " << endeff->X.pos << endl;
	cout << "EEF target pos:  " << target->X.pos << endl;
	cout << "Position error:  " << result.pos_error << endl;
	cout << "Alignment error: " << result.ang_error << endl;

	// check end effector goal position
	if(!withinTolerance(result.pos_error, _pos_tolerance)) {
		result.error_msg.append("Goal position not within tolerance values! ");
		result.status = RESULT_APPROXIMATE;
	}

	// check end effector goal alignment
	if(!withinTolerance(result.ang_error, _ang_tolerance, axes_to_align)) {
		result.error_msg.append("Goal alignment not within tolerance values! ");
		result.status = RESULT_APPROXIMATE;
	}

	// clear the proxies to clean up the UI
	listDelete(_world->proxies);
}

void KomoWrapper::plan(const vector<double> &goal_state, PlanningResult &result)
{
	CHECK(goal_state.size() == _active_joints.size(), "Unable to plan - invalid size of goal state");
	MT::timerStart();

	arr state = _world->getJointState();

	// initialize goal configuration with current state and then assign desired goal values
	arr goal_config = state;
	for(int i = 0; i < _active_joints.size(); ++i) {
		int idx = _active_joints[i]->qIndex;
		goal_config(idx) = goal_state[i];
	}

	// create the motion problem
	MotionProblem MP(*_world);
	TaskCost *c;

	// TaskMap for goal state
	c = MP.addTask("state", new DefaultTaskMap(qItselfTMT, *_world));
	c->setCostSpecs(MP.T-10, MP.T, goal_config, _jointStatePrecision);

	// TaskMap for zero velocity at goal
	c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, *_world));
	c->setCostSpecs(MP.T, MP.T, {0.}, _zeroVelocityPrecision);
	c->map.order = 1; //make this a velocity variable!

	// Constraint to enforce joint limits on all time slices
	LimitsConstraint *lc = new LimitsConstraint();
	lc->margin = 0.005;
	c = MP.addTask("Joint_limits", lc);
	c->setCostSpecs(0, MP.T, {0.}, _jointLimitPrecision);

	// enable collision checking
	TaskCost *colCost = MP.addTask("Collisions", new ProxyTaskMap(allPTMT, {0}, _collisionMargin));
	colCost->setCostSpecs(0, MP.T, {0.}, _collisionPrecision);

	// TaskMap for transition costs
	c = MP.addTask("Transitions", new TransitionTaskMap(*_world));
//	c->map.order = 1; // penalize velocities
	c->map.order = 2; // penalize accelerations
	c->setCostSpecs(0, MP.T, {0.}, 1e0);

	//-- create the Optimization problem (of type kOrderMarkov)
	MotionProblemFunction MF(MP);
	ors::KinematicWorld::setJointStateCount = 0;
	arr traj;
	bool goal_state_reached = false;

	for(int j = 0; j < 10; ++j) {

		cout << "Iteration " << j+1 << endl;

		MP.prefix.clear();
		MP.x0 = state;

		arr x;
		// initialize as interpolation from current state to desired goal state
		sineProfile(x, state, goal_config, MP.T);
		// and then do the optimization
		optConstrained(x, NoArr, Convert(MF), OPT(verbose=0, stopIters=100, maxStep=.5, stepInc=2., allowOverstep=false));
		MP.costReport(false);

		// ensure that all joints are within calculated limits before do collision validation
		ensureJointLimits(*_world, x);

		if(!validateCollisions(*_world, x, result.error_msg)) {
			result.status = RESULT_FAILED;
			cerr << "Validation failed!" << endl;
			return;
		}
		// not necessary any more - just for test purposes...
		if(!validateJointLimits(*_world, x, result.error_msg)) {
			result.status = RESULT_FAILED;
			cerr << "Validation failed!" << endl;
			return;
		}

		traj.append(x);

		state = x[MP.T-1];

		if(check_goal_state(goal_config, state)) {
			goal_state_reached = true;
			break;
		}
	}

	CHECK(traj.d0 > 0, "Trajectory is empty...");

	result.planning_time = MT::timerRead();

	cout << "Optimization process finished" << endl;
	cout << "Optimization time:  " << result.planning_time << endl;
	cout << "SetJointStateCount: " << ors::KinematicWorld::setJointStateCount << endl;

	// consider plan to be successful unless validation shows
	// something different...
	result.status = RESULT_SUCCESS;

	// so far, our path is valid but maybe the goal tolerances are violated...
	pathToTrajectory(result.path, traj);

	cout << "Final state: " << state << endl;
	cout << "State error: " << (goal_config - state) << endl;

	// clear the proxies to clean up the UI
	listDelete(_world->proxies);
}

void KomoWrapper::display(bool block, const char *msg)
{
	_world->watch(block, msg);
}

void KomoWrapper::display(const trajectory_msgs::JointTrajectory &trajectory)
{
	arr state;
	_world->getJointState(state);

	for (int i = 0; i < trajectory.points.size(); ++i) {
		for(int j = 0; j < _active_joints.size(); ++j) {
			int jnt_idx = _world->getJointByName(_active_joints[j]->name)->qIndex;
			double pos = trajectory.points[i].positions[j];
			state(jnt_idx) = pos;
		}
		_world->setJointState(state);
		_world->watch(false, STRING("(time " << std::setw(3) << i <<'/' << trajectory.points.size() <<')').p);
		MT::wait(0.01);
	}
}

void KomoWrapper::allowContact(const char *link, bool allow)
{
	ors::Shape *shape = _world->getShapeByName(link);
	if(shape) {
		shape->cont = !allow;
	} else {
		cerr << "Unable to find shape with name '" << string(link) << "' within model." << endl;
	}
}

void KomoWrapper::pathToTrajectory(trajectory_msgs::JointTrajectory &traj, const arr &path)
{
	// set joint names
	for (ors::Joint *jnt : _active_joints) {
		traj.joint_names.push_back(string(jnt->name));
//		cout << joint_names(i) << endl;
	}

	// create waypoints
	int num_points = path.d0-1;
	traj.points.resize(num_points);
	for (int i = 0; i < num_points; ++i) {
		const arr &wp = path[i];
		CHECK(wp.N>=traj.joint_names.size(), "Invalid path for given group!");
		trajectory_msgs::JointTrajectoryPoint &point = traj.points[i];
		for (ors::Joint *jnt : _active_joints) {
			point.positions.push_back(wp(jnt->qIndex));
		}
	}
}

}
