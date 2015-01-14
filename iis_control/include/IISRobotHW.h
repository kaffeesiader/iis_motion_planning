#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>
#include <map>
#include <XmlRpc.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

using namespace hardware_interface;
using namespace std;

/**
 * Container holding all information required for a single joint in the model.
 */
struct Joint {
	Joint() :position(0), velocity(0), effort(0), targetPosition(0) {}

	/* The name as configured in the URDF */
	string unique_name;
	/* The name in the existing interface */
	string internal_name;

	// current values (readt from hardware)
	double position;
	double velocity;
	double effort;
	// commanded values (set by controllers)
	double targetPosition;
};

typedef boost::shared_ptr<Joint> JointPtr;

/**
 * The JointStateAdapter is responsible for updating the current state of a set of robot joints.
 * It subscribes to the sensor_msgs/JointState topic and refreshes the current position,
 * velocity and effort values accordingly. It also publishes the commanded values to the given
 * topic using std_msgs/Float64MultiArray.msg
 */
class JointStateAdapter {

private:
	ros::Subscriber sub_;
	ros::Publisher pub_;
	map<string, JointPtr> joints_;
	JointStateInterface *state_interface_;
	JointCommandInterface *command_interface_;

    std_msgs::Float64MultiArray pos_msg_;
    vector<string> joint_names_;

	bool initialized_;
	bool readonly_;
	bool muted_;
    string jnt_name_prefix;
	/**
	 * Callback for the JointState topic subscription
	 * @param msg
	 */
	void JointStateCB(const sensor_msgs::JointStatePtr &msg);
	/**
	 * Register the joint handles in the hardware interfaces according to the initial JointState.
	 * No positions will be published until this method is called
	 *
	 * @param initialState
	 */
	void initialize(const sensor_msgs::JointState &initialState);

public:
	JointStateAdapter(ros::NodeHandle &nh, JointStateInterface *state_interface, JointCommandInterface *cmd_interface);
	virtual ~JointStateAdapter();

	/**
	 * Publishes the commanded values to the underlying topic.
	 */
	void Update();
	/**
	 * Returns true if this state adapter is initialized, that means it received it's initial
	 * JointState message and has registered all the JointHandles in the hardware interfaces
	 * @return
	 */
	bool isInitialized();
	/**
	 * Returns true, if this state adapter was configured as readonly, that means it just registers
	 * the current joint positions, but it will not publish the commanded values.
	 *
	 * @return
	 */
	bool isReadOnly();
	/**
	 * Switch the muted state of this JointStateAdapter.
	 * If a JointStateAdapter is muted, it will not publish commanded values.
	 * @param muted
	 */
	void setMuted(bool muted);
};

typedef boost::shared_ptr<JointStateAdapter> JointStateAdapterPtr;

/**
 * Implementation of the hardware interface, needed by the controller manager.
 * It maintains a list of all joints in the model and the currently active Adapters.
 */
class IISRobotHW : public RobotHW {

public:
	IISRobotHW();
	virtual ~IISRobotHW();

	/**
	 * Initializes IISRobotHW with given private and public nodehandles
	 * @param hw_nh Reference to a private NodeHandle for retrieving the configuration parameters
	 */
	bool initialize(ros::NodeHandle &hw_nh);

	/**
	 * Force state adapters to publish the commanded values
	 */
	void publish();
	void setReadOnly(bool read_only);
	bool initialStateReceived();

private:

	bool initialized_;
	bool read_only_;

	JointStateInterface jnt_state_interface_;
	PositionJointInterface jnt_pos_interface_;

	vector<JointStateAdapterPtr> jntStateAdapters_;
};

