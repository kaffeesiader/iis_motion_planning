/*
 * IISRobotHW.cpp
 *
 *  Created on: Oct 16, 2013
 *      Author: martin
 */

#include <sstream>
#include <IISRobotHW.h>
#include <XmlRpc.h>

using namespace hardware_interface;
using namespace std;
using namespace XmlRpc;

IISRobotHW::IISRobotHW() :initialized_(false) {
}

IISRobotHW::~IISRobotHW() {
}
/**
 * Initialize the fake hardware.
 * Create a list of joints in the model and register hardware interfaces
 *
 * @param hw_nh Reference to private Nodehandle
 */
bool IISRobotHW::initialize(ros::NodeHandle &hw_nh) {

	if (initialized_)
		return true;

	ROS_INFO("Initializing state adapters");

	if (!hw_nh.hasParam("adapter_list")) {
		ROS_ERROR("IISRobotHW: No adapter_list specified.");
		return false;
	}

	XmlRpcValue adapter_list;
	hw_nh.getParam("adapter_list", adapter_list);
	if (adapter_list.getType() != XmlRpcValue::TypeArray) {
		ROS_ERROR("IISRobotHW: adapter_list should be specified as an array");
		return false;
	}

	/* actually create each adapter */
	for (int i = 0; i < adapter_list.size(); ++i) {

		// try to launch a JointStateAdapter for the given list_item...
		try {
			string name = adapter_list[i];

			ROS_INFO("Loading JointStateAdapter '%s'...", name.c_str());

			// create a private Nodehandle for each adapter
			ros::NodeHandle a_nh(hw_nh, name);
			JointStateAdapterPtr adapter(new JointStateAdapter(a_nh, &jnt_state_interface_, &jnt_pos_interface_));

			ROS_INFO("Waiting for initial state...");
			int attempts = 0;

			// wait for initial state until a timeout...
			while(attempts < 20) {
				ros::spinOnce();

				if(!adapter->isInitialized()) {
					ros::WallDuration(0.1).sleep();
					attempts++;
				} else {
					break;
				}
			}

			if(!adapter->isInitialized()) {
				ROS_ERROR("Unable to receive initial state before timeout. Dropping this state adapter!");
			} else {
				jntStateAdapters_.push_back(adapter);
			}
		} catch (string &e) {
			ROS_WARN("Error loading JointStateAdapter: '%s'", e.c_str());
		}
	}

	// quit, if no state adapter could be loaded
	if(!jntStateAdapters_.size() > 0) {
		ROS_ERROR("Unable to load any state interface - please check configuration!");
		return false;
	}

	ROS_INFO("Registering hardware interfaces");

	// register our Interfaces in the RobotHW
	this->registerInterface(&jnt_state_interface_);
	this->registerInterface(&jnt_pos_interface_);

	initialized_ = true;

	ROS_INFO("Hardware adapters initialized");
	return true;
}

bool IISRobotHW::initialStateReceived() {

	for (size_t i = 0; i < jntStateAdapters_.size(); ++i) {
		if(!jntStateAdapters_[i]->isInitialized()){
			return false;
		}
	}

	return true;
}

/**
 * Force Command adapters to publish JointPositions
 */
void IISRobotHW::publish() {

	for (size_t i = 0; i < jntStateAdapters_.size(); ++i) {
		jntStateAdapters_[i]->Update();
	}

}

/* -------------------- JointStateAdapter ----------------------- */

JointStateAdapter::JointStateAdapter(ros::NodeHandle &nh, JointStateInterface *state_itf, JointCommandInterface *cmd_itf):
		initialized_(false), readonly_(false) {

	string state_topic;
	// Create public Nodehandle for topic subscriptions and use private one
	// just for retrieving the private parameters
	ros::NodeHandle public_nh;

	if(!nh.getParam("joint_state_topic", state_topic)) {
		// if no topic name is configured this adapter is unable to work!
		string msg = "Unable to launch JointStateAdapter - no state topic name defined!";
		throw msg;
	}

	if(!nh.getParam("readonly", readonly_)) {
		ROS_INFO("Parameter 'readonly' not set - assuming that adapter is not readonly.");
	}

	if(!readonly_) {
		string cmd_topic;

		if(!nh.getParam("joint_command_topic", cmd_topic)) {
			string msg = "Unable to launch JointStateAdapter - adapter is not readonly and no command topic name defined!";
			throw msg;
		}

        // read the jointnames
        if (!nh.hasParam("joints")) {
            string msg = "Unable to launch JointStateAdapter: joints parameter not specified.";
            throw msg;
        }
        XmlRpcValue joint_list;
        nh.getParam("joints", joint_list);
        if (joint_list.getType() != XmlRpcValue::TypeArray) {
            string msg = "Unable to launch JointStateAdapter: joints parameter should be specified as an array";
            throw msg;
        }
        for (size_t i = 0; i < joint_list.size(); ++i) {
            joint_names_.push_back(joint_list[i]);
        }

		// create a publisher for the JointPosition messages if not readonly...
        pub_ = public_nh.advertise<std_msgs::Float64MultiArray>(cmd_topic, 1);
		string topic_name = pub_.getTopic();
		ROS_INFO("JointStateAdapter publishes to topic '%s'", topic_name.c_str());
	}

	if(!nh.getParam("joint_name_prefix", this->jnt_name_prefix)) {
		ROS_WARN("Parameter 'joint_name_prefix' not set - assuming that there is no prefix.");
	}

	state_interface_ = state_itf;
	command_interface_ = cmd_itf;

	sub_ = public_nh.subscribe(state_topic, 1, &JointStateAdapter::JointStateCB, this);
	// receive absolute topic name (in case that node runs under a different namespace...)
	string topic_name = sub_.getTopic();

	ROS_INFO("JointStateAdapter listens on topic '%s'", topic_name.c_str());

}

void JointStateAdapter::JointStateCB(const sensor_msgs::JointStatePtr& msg) {
	if(!initialized_) {
		initialize(*msg.get());
	} else {
		// update the values according to the message
		for (size_t i = 0; i < msg->name.size(); ++i) {
			std::string jntName = msg->name[i];
			JointPtr jnt = joints_[jntName];

			if (!jnt) {
				ROS_ERROR("Error retrieving jointstate - no joint with name '%s' exists", jntName.c_str());
			} else {
				// always check if value at position i exists...
				if (msg->position.size() > i) {
					jnt->position = msg->position[i];
				}
				if (msg->velocity.size() > i) {
					jnt->velocity = msg->velocity[i];
				}
				if (msg->effort.size() > i) {
					jnt->effort = msg->effort[i];
				}
			}
		}
	}
}

void JointStateAdapter::initialize(const sensor_msgs::JointState &initialState) {
    ROS_INFO("Initializing state adapter");

	for (size_t i = 0; i < initialState.name.size(); ++i) {
		// this is the joint name without prefix
		std::string name = initialState.name[i];
		// this is the joint name compliant to our urdf
		std::string fullName = jnt_name_prefix + name;
		// create a joint structure for each joint in the initial state
		JointPtr jnt(new Joint());

		jnt->unique_name = fullName;

		// set initial values (check first if value at position exists...)
		if (initialState.position.size() > i) {
			jnt->position = initialState.position[i];
		}
		if (initialState.velocity.size() > i) {
			jnt->velocity = initialState.velocity[i];
		}
		if (initialState.effort.size() > i) {
			jnt->effort = initialState.effort[i];
		}
		// set initial target position to current position
		jnt->targetPosition = jnt->position;

		ROS_INFO("Registering joint '%s'", jnt->unique_name.c_str());

		// register the joint handles in the hardware interfaces
		JointStateHandle state_handle(jnt->unique_name, &jnt->position, &jnt->velocity, &jnt->effort);
		JointHandle pos_handle(state_handle, &jnt->targetPosition);

		state_interface_->registerHandle(state_handle);
		command_interface_->registerHandle(pos_handle);

		joints_[name] = jnt;
//        joint_names_.push_back(name);
	}

	initialized_ = true;

	ROS_INFO("State adapter initialized");
}

void JointStateAdapter::Update() {
	// do nothing if this adapter is readonly
	if(readonly_)
		return;

    pos_msg_.data.clear();

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        string jnt_name = joint_names_[i];
	JointPtr jnt = joints_[jnt_name];
        if(!jnt) {
            ROS_WARN_STREAM("No joint with name " << jnt_name << " exists!");
        } else {
            pos_msg_.data.push_back(jnt->targetPosition);
        }
//        pos_msg_.data.push_back(jnt->targetPosition);
	}

	pub_.publish(pos_msg_);
}

bool JointStateAdapter::isInitialized() {
	return initialized_;
}

bool JointStateAdapter::isReadOnly() {
	return readonly_;
}

JointStateAdapter::~JointStateAdapter() {
}

