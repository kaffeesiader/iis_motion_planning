#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iis_robot.h>

using namespace std;
using namespace ros;

namespace iis_komo {


class RobotInterface {

	typedef boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> ControllerClientPtr;

public:

	RobotInterface(NodeHandle &nh);
	~RobotInterface() {}

	sensor_msgs::JointState getCurrentState();
	bool execute(IISRobot::PlanninGroup group, trajectory_msgs::JointTrajectory &trajectory);

private:

	Publisher _pub_left_arm_move;
	Publisher _pub_right_arm_move;

	Subscriber _sub_left_arm_state;
	Subscriber _sub_right_arm_state;
	Subscriber _sub_left_sdh_state;
	Subscriber _sub_right_sdh_state;

	IISRobotState _current_state;

	ControllerClientPtr _left_arm_client;
	ControllerClientPtr _right_arm_client;

	void CBLeftArmState(const sensor_msgs::JointStatePtr &msg);
	void CBRightArmState(const sensor_msgs::JointStatePtr &msg);
	void CBLeftSdhState(const sensor_msgs::JointStatePtr &msg);
	void CBRightSdhState(const sensor_msgs::JointStatePtr &msg);

};

}


#endif // ROBOT_INTERFACE_H
