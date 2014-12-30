#include <Ors/ors_swift.h>

#include <io_helpers.h>
#include <utils.h>
#include <komo_helpers.h>

using namespace std;

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
				cout << boost::format("Joint '%d' in point '%d' closed to lower limit - offset: %.f ") % j % i % (wp(j)-lo) << endl;
			else if((hi-wp(j)) < margin)
				cout << boost::format("Joint '%d' in point '%d' closed to upper limit - offset: %.f ") % j % i % (hi-wp(j)) << endl;

		}
	}

}

void run_tests(ors::KinematicWorld &w, const vector<geometry_msgs::Pose> &test_set, const char *link)
{
	ors::Shape *target = w.getShapeByName("target");
	CHECK(target, "No shape with name 'target' found in model.");

	arr xT = w.getJointState();

	double maxPosError = 0.01, maxAngError = 0.05;
	int totalPositive = 0;

	ors::Vector posError, angError, posTol, angTol;
	posTol.set(maxPosError, maxPosError, maxPosError);
	angTol.set(maxAngError, maxAngError, maxAngError);

	for(size_t i = 0; i < test_set.size(); ++i) {
		cout << "Testing pose " << i+1 << " out of " << test_set.size() << endl;
		const geometry_msgs::Pose &pose = test_set[i];
		ors::Transformation goal;
		poseToTransformation(pose, goal);
		// place the target marker (for visualization only...)
		target->rel = goal;
		w.calc_fwdPropagateFrames();

		MT::timerStart();

		double totalCosts = 0;
		MT::timerStart();

		bool success = false;

		for(int j = 0; j < 10; ++j) {

			cout << "Iteration " << j+1 << endl;

			totalCosts += optimizeEndpose(xT, w, link, target->name);

			string error;
			if(!validateCollisions(w, xT, error)) {
				cout << "Validation error: " << error << endl;
				break;
			}

			w.setJointState(xT);

			computePositionError(w, link, target->name, posError);
			computeAlignmentError(w, link, target->name, angError);

			if(withinTolerance(posError, posTol) && withinTolerance(angError, angTol)) {
				cout << "Goal reached!" << endl;
				MT::timerPause();
				w.watch(false, "Resulting configuration");
				success = true;
				break;
			}
		}

		double time = MT::timerRead();
		totalPositive += success;

		cout << "Position error    : " << posError << endl;
		cout << "Angular error     : " << angError << endl;

		string result = (success ? "successful" : "failed");

		cout << "Optimization time: " << time << endl;
		cout << "Reported costs   : " << totalCosts << endl;
		cout << "Pose " << i+1 << " " << result << endl << endl;
	}

	cout << "Test run completed." << endl;
	cout << totalPositive << " solutions found" << endl;
	cout << (int)test_set.size() - totalPositive << " planning attempts failed!" << endl;

}

int main(int argc, char *argv[])
{
	if(argc < 3) {
		ROS_ERROR("Usage: %s INPUT_FN EEF_LINK [optional TEST_CNT = 20]", argv[0]);
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
//	srand(time(NULL));
	srand(10);

	vector<geometry_msgs::Pose> test_set;
	for (int i = 0; i < test_count; ++i) {
		int max = (int)poses.size();
		int index = rand() % max;
		test_set.push_back(poses[index]);
		poses.erase(poses.begin()+index);
	}

	MT::String name = MT::getParameter<MT::String>("KOMO/robot_config");
	ors::KinematicWorld w(name);

	run_tests(w, test_set, link);

	return EXIT_SUCCESS;
}



