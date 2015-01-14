#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Optim/optimization.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>

#include <io_helpers.h>
#include <komo_helpers.h>
#include <utils.h>
#include <boost/format.hpp>


using namespace std;

void setup_MP(MotionProblem &MP, const char *link, const char *target, bool useHoming) {

	double posPrec = 1e4;
	double alignPrec = 1e6; // original 1e3
	double collPrec = 1e0;
	double collMarg = 0.04;
	double jntLimitPrec = 1e1;
	double zeroVelPrec = 1e1;

	listDelete(MP.taskCosts);
	cout << "Setting up MP " << (useHoming ? "with" : "without") << " homing." << endl;

	ors::KinematicWorld &w = MP.world;
	TaskCost *c;

	// TaskMap for end effector position
	c = MP.addTask("position", new DefaultTaskMap(posTMT, w, link, NoVector, target, NoVector));
	c->setCostSpecs(MP.T, MP.T, {0.}, posPrec);

	for(uint i=0;i<3;i++) {
		ors::Vector axis;
		axis.setZero();
		axis(i) = 1.;
		c = MP.addTask(STRING("allign_" << i), new DefaultTaskMap(vecAlignTMT, w, link, axis, "target", axis));
		c->setCostSpecs(MP.T, MP.T, {1.}, alignPrec);
	}

	if(useHoming) {
		// TaskMap for goal state
		c = MP.addTask("state", new DefaultTaskMap(qItselfTMT, w));
		c->setCostSpecs(0, MP.T, {0.}, 1e-2);
	}

	// TaskMap for zero velocity at goal
	c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, w));
	c->setCostSpecs(MP.T, MP.T, {0.}, zeroVelPrec);
	c->map.order = 1; //make this a velocity variable!

	// Constraint to enforce joint limits on all time slices
	LimitsConstraint *lc = new LimitsConstraint();
	lc->margin = 0.005;
	c = MP.addTask("Joint_limits", lc);
	c->setCostSpecs(0, MP.T, {0.}, jntLimitPrec);

	// enable collision checking
	TaskCost *colCost = MP.addTask("Collisions", new ProxyTaskMap(allPTMT, {0}, collMarg));
	colCost->setCostSpecs(MP.T, MP.T, {0.}, collPrec);

	// TaskMap for transition costs
	c = MP.addTask("Transitions", new TransitionTaskMap(w));
//	c->map.order = 1; // penalize velocities
	c->map.order = 2; // penalize accelerations
	c->setCostSpecs(0, MP.T, {0.}, 1e0);
}

void check_limits(ors::KinematicWorld &w, arr x, double margin) {
	// get the joint limits as configured in the ors description
	arr limits = w.getLimits();

	// validation steps neccessary for each single point
	for(int i = 0; i < x.d0 - 1; ++i) {
		arr wp = x[i];

		for (int j = 0; j < wp.d0; ++j) {
			double hi = limits(j,1);
			double lo = limits(j,0);

			// ensure that there actually is a limit, i.e. upper-lower > 0
			if(hi == lo)
				continue;

			if((wp(j)-lo) < margin)
				cout << boost::format("Joint '%d' in point '%d' closed to lower limit %.3f - value: %.3f ") % j % i % lo % wp(j) << endl;
			else if((hi-wp(j)) < margin)
				cout << boost::format("Joint '%d' in point '%d' closed to upper limit %.3f - value: %.3f ") % j % i % hi % wp(j) << endl;

		}
	}

}

void run_tests(ors::KinematicWorld &w, const vector<geometry_msgs::Pose> &test_set, const char *link)
{
	ors::Shape *target = w.getShapeByName("target");
	CHECK(target, "No shape with name 'target' found in model.");

	MotionProblem MP(w);
	MP.loadTransitionParameters();
	//-- create the Optimization problem (of type kOrderMarkov)
	MotionProblemFunction MF(MP);

	arr initialState = w.getJointState(), state = initialState;
	ors::Transformation goal;

	double maxPosError = 0.01, maxAngError = 0.05, totalTime = 0;
	int totalPositive = 0, limitViolations = 0, collisions = 0;

	ors::Vector posError, angError, posTol, angTol;
	posTol.set(maxPosError, maxPosError, maxPosError);
	angTol.set(maxAngError, maxAngError, maxAngError);

	for(size_t i = 0; i < test_set.size(); ++i) {
		cout << "Testing pose " << i+1 << " out of " << test_set.size() << endl;
		const geometry_msgs::Pose &pose = test_set[i];
		poseToTransformation(pose, goal);
		// place the target marker (for visualization only...)
		target->rel = goal;

		w.calc_fwdPropagateFrames();
		w.watch();

		double costs, totalCosts = 0;
		bool success = false;
		MT::timerStart();
		arr traj;

		state = initialState;

		for(int j = 0; j < 2; ++j) {

			cout << "Iteration " << j+1 << endl;

			MP.prefix.clear();
			MP.x0 = state;
			setup_MP(MP, link, target->name, false);

			arr x = replicate (MP.x0, MP.T+1);

			optConstrained(x, NoArr, Convert(MF), OPT(verbose=0, stopIters=100, maxStep=.5, stepInc=2., allowOverstep=false, fmin_return=&costs));

			traj.append(x);
			totalCosts += costs;

			string error;
			if(!validateCollisions(w, x, error)) {
				cout << "Validation error: " << error << endl;
				collisions++;
				break;
			}
			if(!validateJointLimits(w, x, error)) {
				cout << "Validation error: " << error << endl;
				limitViolations++;
				break;
			}

			state = x[MP.T-1];
			w.setJointState(state);
			computePositionError(w, link, target->name, posError);
			computeAlignmentError(w, link, target->name, angError);

			cout << "Position error    : " << posError << endl;
			cout << "Angular error     : " << angError << endl;

			check_limits(w, x, 0.01);

			if(withinTolerance(posError, posTol) &&
			   withinTolerance(angError, angTol))
			{
				MT::timerPause();
				success = true;
				totalPositive++;
				// only set initial state to current state if planning was successful
				initialState = state;
				listDelete(w.proxies);
				displayTrajectory(traj, -1, w, "Planning result", 0.01);
				break;
			}
		}

		double time = MT::timerRead();
		totalTime += time;
		string result = (success ? "successful" : "failed");

		cout << "Optimization time: " << time << endl;
		cout << "Reported costs   : " << totalCosts << endl;
//		cout << "State: " << state << endl;
		cout << "Pose " << i+1 << " " << result << endl << endl;
//		displayTrajectory(traj, -1, w, "Planning result", 0.01);
	}

	cout << "Test run completed." << endl;
	cout << totalPositive << " solutions found" << endl;
	cout << (int)test_set.size() - totalPositive << " planning attempts failed!" << endl;
	cout << "Limit violations: " << limitViolations << endl;
	cout << "Collisions      : " << collisions << endl;
	cout << "Average planning time: " << ((double)totalTime / test_set.size()) << endl;

}

int main(int argc, char *argv[])
{
	if(argc < 3) {
		ROS_ERROR("Usage: %s INPUT_FN EEF_LINK [optional TEST_CNT = 20]", argv[0]);
		return EXIT_FAILURE;
	}

	char *input_fn = argv[1];
	char *link = argv[2];

	int test_count = 20;
	if(argc > 3) {
		test_count = atoi(argv[3]);
	}

	ROS_INFO("Starting test run for input file '%s' for EEF_LINK '%s'", input_fn, link);

	// load test poses from file
	vector<geometry_msgs::Pose> poses;
	if(!read_poses_from_file(input_fn, poses)) {
		ROS_ERROR("Loading test poses failed!");
		return EXIT_FAILURE;
	}

	// ensure that test_count is not greater than count of test poses
	test_count = test_count > poses.size() ? poses.size() : test_count;

	// generate random test set
	srand(time(NULL));
//	srand(12345);

	vector<geometry_msgs::Pose> test_set;
	for (int i = 0; i < test_count; ++i) {
		int max = (int)poses.size();
		int index = rand() % max;
		test_set.push_back(poses[index]);
		poses.erase(poses.begin()+index);
	}

	ors::KinematicWorld w("../data/iis_robot.kvg");

	run_tests(w, test_set, link);

	return EXIT_SUCCESS;
}


