#ifndef IIS_ROBOT_H
#define IIS_ROBOT_H

#include <Core/array.h>
#include <vector>

namespace iis_komo {

struct IISRobot {

	typedef double ArmState[7];
	typedef double HandState[7];
	typedef arr Path;

	enum PlanninGroup {
		LeftArm  = 0,
		RightArm = 1,
		LeftSDH  = 2,
		RightSDH = 3
	};

	static MT::Array<const char*> left_arm_get_jointnames() {
	  return {
			"left_arm_0_joint",
			"left_arm_1_joint",
			"left_arm_2_joint",
			"left_arm_3_joint",
			"left_arm_4_joint",
			"left_arm_5_joint",
			"left_arm_6_joint"
	  };
	}

	static MT::Array<const char*> right_arm_get_jointnames() {
	  return {
			"right_arm_0_joint",
			"right_arm_1_joint",
			"right_arm_2_joint",
			"right_arm_3_joint",
			"right_arm_4_joint",
			"right_arm_5_joint",
			"right_arm_6_joint"
	  };
	}

	static MT::Array<const char*> left_sdh_get_jointnames() {
	  return {
			"left_sdh_knuckle_joint",
			"left_sdh_finger_12_joint",
			"left_sdh_finger_13_joint",
			"left_sdh_thumb_2_joint",
			"left_sdh_thumb_3_joint",
			"left_sdh_finger_22_joint",
			"left_sdh_finger_23_joint"
	  };
	}

	static MT::Array<const char*> right_sdh_get_jointnames() {
	  return {
			"right_sdh_knuckle_joint",
			"right_sdh_finger_12_joint",
			"right_sdh_finger_13_joint",
			"right_sdh_thumb_2_joint",
			"right_sdh_thumb_3_joint",
			"right_sdh_finger_22_joint",
			"right_sdh_finger_23_joint"
	  };
	}

	static MT::Array<const char*> get_jointnames_from_group(const PlanninGroup agent) {
		switch (agent) {
		case LeftArm:
			return left_arm_get_jointnames();
		case RightArm:
			return right_arm_get_jointnames();
		case LeftSDH:
			return left_sdh_get_jointnames();
		case RightSDH:
			return right_sdh_get_jointnames();
		}
	}

	static const arr get_arm_velocity_limits() {
		// values only 10% of real velocity limits...
		return {
			0.191986,
			0.191986,
			0.226893,
			0.226893,
			0.226893,
			0.314159,
			0.314159
		};
	}

	static const arr get_arm_joint_limits() {
		return {
			2.96705972839,
			2.09439510239,
			2.96705972839,
			2.09439510239,
			2.96705972839,
			2.09439510239,
			2.96705972839
		};
	}

};

struct IISRobotState {

	IISRobot::ArmState left_arm;
	IISRobot::ArmState right_arm;
	IISRobot::HandState left_sdh;
	IISRobot::HandState right_sdh;

};

}

#endif // IIS_ROBOT_H
