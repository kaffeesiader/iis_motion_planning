#include <Ors/ors.h>
#include <Motion/komo.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <komo_interface.h>


#define JOINT_STATES_TOPIC "/simulation/right_arm/joint_control/get_state"
#define JOINT_MOVE_TOPIC "/simulation/right_arm/joint_control/move"


using namespace ros;
using namespace std;



int main(int argc,char** argv) {
    MT::initCmdLine(argc,argv);

	ros::init(argc, argv, "iis_komo_test");
	ros::AsyncSpinner spinner(1);
	spinner.start();


	spinner.stop();
    return EXIT_SUCCESS;
}


