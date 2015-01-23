#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Ors/ors_sceneGui.h>
#include <Optim/optimization.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>
#include <string>
#include <komo_helpers.h>

using namespace std;

int main(int argc, char *argv[])
{
	ors::KinematicWorld w("../data/iis_robot.kvg");

//	arr state(w.getJointStateDimension());
//	state(2) = 1.0;
//	w.setJointState(state);
//	w.swift().step(w,false);
//	cout << "Size: " << w.proxies.N << endl;
//	for (ors::Proxy *p : w.proxies) if(p->d <= 0.08) {
//		cout << w.shapes(p->a)->name << " vs. " << w.shapes(p->b)->name << endl;
//		cout << "Distance: " << p->d << endl;
//	}

	MotionProblem MP(w);
	TaskCost *c;
	c = MP.addTask("Collision", new CollisionConstraint());
	c->setCostSpecs(0, MP.T, {0.}, 1e0);

	uintA shapes;
	c = MP.addTask("Collision", new ProxyTaskMap(allPTMT, shapes, 0.02));
	c->setCostSpecs(0, MP.T, {0.}, 1e0);

	arr x1 = ARR(2.,3.,4.);
	arr x2 = x1 * 3.;
	x2 = x1 / 3.;

	ors::Body *target = w.getBodyByName("target");

	target->X.pos.x = 0.3;
	target->X.pos.y = 0.2;
	target->X.pos.z = 0.4;


	target->X.rot.x = .707107; //.707107; // 1.0 // 0.92388
	target->X.rot.y = .707107; //.707107; // 0.0 // 0.382683
	target->X.rot.z = 0;		  // 0.0 // 0.0
	target->X.rot.w = 0;		  // 0.0 // 0.0

	w.calc_fwdPropagateFrames();

	const char *link = "right_eef";

	//-- optimize
	arr x;
	ors::Vector posError, rotError;
	MT::timerStart();
	double cost = optimizeEndpose(x, w, link, target->name);
	double time = MT::timerRead();

	computePositionError(w, link, target->name, posError);
	computeAlignmentError(w, link, target->name, rotError);

	cout << "Optimization time: " << time << endl;
	cout << "Cost             : " << cost << endl;
	cout << "Position error   : " << posError << endl;
	cout << "Alignment error  : " << rotError << endl;

	displayState(x, w, "First state");

	return EXIT_SUCCESS;
}


