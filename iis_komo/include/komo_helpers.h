#ifndef KOMO_HELPERS_H
#define KOMO_HELPERS_H

#include <boost/format.hpp>

#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Optim/optimization.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>

using namespace std;

// could also be used to find an IK solution as it only computes the final state...
double keyframeOptimizer(arr& x, MotionProblem& MP, bool x_is_initialized, uint verbose) {

	MotionProblem_EndPoseFunction MF(MP);

	if (!x_is_initialized) x=MP.x0;

	double cost;

	optNewton(x, Convert(MF), OPT(fmin_return=&cost, verbose=verbose, stopIters=200, damping=1e-0, maxStep=.5, stopTolerance=1e-2));

	return cost;
}

double optimizeEndpose(arr &xT, ors::KinematicWorld &w, const char *link, const char *target, bool allowCollision = true) {

	double posPrec = 1e4;
	double alignPrec = 1e5; // original 1e3
	double collPrec = 1e1;
	double collMarg = 0.02;
	double jntLimitPrec = 1e1;

	//-- set up the MotionProblem
	arr hrate(w.getJointStateDimension());
	for (int i = 0; i < hrate.N; ++i) {
		hrate(i) = 0.0;
	}

	MotionProblem MP(w);
	MP.H_rate_diag = hrate;

	TaskCost *c;

	// TaskMap for end effector position
	c = MP.addTask("position", new DefaultTaskMap(posTMT, w, link, NoVector, target, NoVector));
	c->setCostSpecs(MP.T, MP.T, {0.}, posPrec);

	c = MP.addTask("alignX", new DefaultTaskMap(vecAlignTMT, w, link, {1,0,0}, target, {1,0,0}));
	c->setCostSpecs(MP.T, MP.T, {1.}, alignPrec);

	c = MP.addTask("alignY", new DefaultTaskMap(vecAlignTMT, w, link, {0,1,0}, target, {0,1,0}));
	c->setCostSpecs(MP.T, MP.T, {1.}, alignPrec);

	c = MP.addTask("alignZ", new DefaultTaskMap(vecAlignTMT, w, link, {0,0,1}, target, {0,0,1}));
	c->setCostSpecs(MP.T, MP.T, {1.}, alignPrec);

	// TaskMap to enforce joint limits on all time slices
	c = MP.addTask("Joint_limits", new DefaultTaskMap(qLimitsTMT));
	c->setCostSpecs(0, MP.T, {0.}, jntLimitPrec);

	// Constraint to enforce joint limits on all time slices
//	LimitsConstraint *lc = new LimitsConstraint();
//	lc->margin = 0.005;
//	c = MP.addTask("Joint_limits", lc);
//	c->setCostSpecs(0, MP.T, {0.}, jntLimitPrec);

	if(!allowCollision) {
		// enable collision checking
		c = MP.addTask("Collisions", new ProxyTaskMap(allPTMT, {0}, collMarg));
		c->setCostSpecs(0, MP.T, {0.}, collPrec);
	}

	return keyframeOptimizer(xT, MP, false, 1);
}

void computePositionError(const ors::Shape &eef, const ors::Shape &target, ors::Vector &error) {
	CHECK(&eef && &target, "One of the provided shapes does not exist!");
	error = eef.X.pos - target.X.pos;
}

void computePositionError(ors::KinematicWorld &w, const char *eef, const char *target, ors::Vector &error) {
	ors::Shape *_eef = w.getShapeByName(eef);
	ors::Shape *_target = w.getShapeByName(target);
	return computePositionError(*_eef, *_target, error);
}

void computeAlignmentError(const ors::Shape &eef, const ors::Shape &target, ors::Vector &error, int axes = 7) {
	CHECK(&eef && &target, "One of the provided shapes does not exist!");

	ors::Vector tx,ty,tz, ex, ey, ez;

	target.X.rot.getX(tx);
	target.X.rot.getY(ty);
	target.X.rot.getZ(tz);
	eef.X.rot.getX(ex);
	eef.X.rot.getY(ey);
	eef.X.rot.getZ(ez);
	error.x = tx.angle(ex);
	error.y = ty.angle(ey);
	error.z = tz.angle(ez);

	// clear error value on axes we do not have to consider
	for(int i = 0; i < 3; ++i) if(!(axes & (1 << i))) {
		error(i) = 0.;
	}
}

void computeAlignmentError(ors::KinematicWorld &w, const char *eef, const char *target, ors::Vector &error) {
	ors::Shape *_eef = w.getShapeByName(eef);
	ors::Shape *_target = w.getShapeByName(target);
	return computeAlignmentError(*_eef, *_target, error);
}

bool withinTolerance(ors::Vector &error, ors::Vector &tolerance, int axes)
{
	for(uint i=0;i<3;i++) if(axes & (1 << i)) {
		if(fabs(error(i)) > tolerance(i))
			return false;
	}
	return true;
}

bool withinTolerance(ors::Vector &error, ors::Vector &tolerance)
{
	return withinTolerance(error, tolerance, 7);
}

/**
 * Checks if each joint position in given waypoint meets the configured limits
 *
 * @param wp		The waypoint to check
 * @param limits	The limits to enforce
 * @return			True if no joint limit violation was detected
 */
bool validateJointLimits(arr wp, arr limits, string &error_msg) {
	CHECK(wp.d0 == limits.d0, "Wrong dimensions!");
	for (int i = 0; i < wp.d0; ++i) {
		double hi = limits(i,1);
		double lo = limits(i,0);

		// ensure that there actually is a limit, i.e. upper-lower > 0
		if(hi == lo)
			continue;

		if(wp(i) < lo || wp(i) > hi) {
			error_msg.append(STRING(boost::format("Joint limit violated for joint '%d' - value: %.3f\n") % i % wp(i)));
			return false;
		}
	}
	return true;
}

/**
  Check if all joint positions within given trajectory or configuation are within the
  configured limits
 *
 * @param w The KinematicWorld instance to use
 * @param x The state to check. x can be either a single waypoint (nd=1) or a list of
 *			waypoints(nd=2)
 * @return  True, if no configuration violates the joint limits
 */
bool validateJointLimits(ors::KinematicWorld &w, arr x, string &error_msg) {
	// get the joint limits as configured in the ors description
	arr limits = w.getLimits();
	CHECK(x.nd > 0, "Given value is neither a trajectory nor a waypoint!");

	if(x.nd == 2) { // trajectory (list of waypoints)
		// validation steps neccessary for each single point
		for(int i = 0; i < x.d0 - 1; ++i) {
			arr pt = x[i];
			if(!validateJointLimits(pt, limits, error_msg)) {
				return false;
			}
		}
		return true;

	} else { // single waypoint
		return validateJointLimits(x, limits, error_msg);
	}
}
/**
 * Correct joint limit violations in given trajectory. Ensures that each joint position in
 * all waypoints is within configured limits.
 *
 * @param w			The KinematicWorld to use
 * @param x			The trajectory to check
 */
void ensureJointLimits(ors::KinematicWorld &w, arr &x) {
	// get the joint limits as configured in the ors description
	arr limits = w.getLimits();
	// steps neccessary for each single point
	for(int i = 0; i < x.d0 - 1; ++i) {
		arr wp = x[i];
		CHECK(wp.d0 == limits.d0, "Wrong dimensions!");
		for (int j = 0; j < wp.d0; ++j) {
			double hi = limits(j,1);
			double lo = limits(j,0);
			// ensure that there actually is a limit, i.e. upper-lower > 0
			if(hi == lo) continue;

			if(wp(j) < lo) {
				cerr << boost::format("Lower joint limit violated for joint %d in waypoint %d - value: %.3f\n") % j % i % wp(j) << endl;
				wp(j) == lo;
				cerr << "corrected!" << endl;
			}

			if(wp(j) > hi){
				cerr << boost::format("Upper joint limit violated for joint %d in waypoint %d - value: %.3f\n") % j % i % wp(j) << endl;
				wp(j) == hi;
				cerr << "corrected!" << endl;
			}
		}
	}
}

/**
 * Check given trajectory or waypoint for detected collisions, using given KinematicWorld
 *
 * @param w			The KinematicWorld to use for collision checking
 * @param x			The Trajectory or waypoint to check
 * @param error_msg	A string that will contain a resulting error message
 * @return			True, if no collision was detected
 */
bool validateCollisions(ors::KinematicWorld &w, const arr &x, string &error_msg)
{
	CHECK(x.nd > 0, "Given value is neither a trajectory nor a waypoint!");
	if(x.nd == 2) { // x is trajectory
		for (int i = 0; i < x.d0-1; ++i) {
			arr pt = x[i];
			w.setJointState(pt);
			// force swift to compute the collision proxies...
			w.swift().step(w);
			// then iterate through the list of proxies and make sure that
			// all distances are greater then zero (or an arbitrary collision margin if necessary...)
			for(ors::Proxy *p : w.proxies)
				if(p->d <= 0) {
					string sA(w.shapes(p->a)->name);
					string sB(w.shapes(p->b)->name);

					error_msg.append(STRING(boost::format("Collision in waypoint %d between '%s' and '%s' detected!\n")
											% i % sA % sB));

//					w.watch(true, error_msg.c_str());

					return false;
				}
		}
		return true;
	} else { // x is single waypoint
		w.setJointState(x);
		// force swift to compute the collision proxies...
		w.swift().step(w);
		// then iterate through the list of proxies and make sure that
		// all distances are greater then zero (or an arbitrary collision margin if necessary...)
		for(ors::Proxy *p : w.proxies)
			if(p->d <= 0) {
				string sA(w.shapes(p->a)->name);
				string sB(w.shapes(p->b)->name);

				error_msg.append(STRING(boost::format("Collision between '%s' and '%s' detected!\n")
										% sA % sB));

				cerr << error_msg << endl;
				w.watch(true, error_msg.c_str());
				return false;
			}
	}
	return true;
}
/**
 * Checks, whether a given goal state was reached
 *
 * @param desired	The desired state
 * @param actual	The actual state
 * @return			True, if actual state is within goal tolerance
 */
bool check_goal_state(const arr &desired, const arr &actual) {
	arr error = desired - actual;
	for (int i = 0; i < desired.d0; ++i) {
		if(fabs(error(i)) > 1e-2)
			return false;
	}
	return true;
}

#endif // KOMO_HELPERS_H
