#include <Ors/ors.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <robot_interface.h>
#include <iis_robot.h>
#include <utils.h>


using namespace ros;
using namespace std;
using namespace iis_komo;

int main(int argc,char** argv) {

	double x = MT::getParameter<double>("KOMO/moveTo/collisionMargin");
	cout << "x: " << x << endl;
	arr test = {1,2,3};
	test = MT::getParameter<arr>("Test/Array");

	cout << "Test: " << test << endl;

	ifstream is("links.txt", std::ifstream::in);
	cout << is.good() << endl;

	MT::Array<MT::String> names;
	names.read(is);
	cout << "Names: " << names << endl;

	return EXIT_SUCCESS;
}


