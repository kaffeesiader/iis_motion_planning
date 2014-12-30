#!/usr/bin/env python
'''
This script adds the agent ids to the joints based on their names. These ids
are used in our code as 'MoveGroups'. With these we can get joint limits, shape
names etc. See share/src/Motion/pr2_heuristics.h.
'''
import re


move_group = {
    # Base
    'fl_caster_rotation_joint': 4,
    'fl_caster_l_wheel_joint': 4,
    'fl_caster_r_wheel_joint': 4,
    'fr_caster_rotation_joint': 4,
    'fr_caster_l_wheel_joint': 4,
    'fr_caster_r_wheel_joint': 4,
    'bl_caster_rotation_joint': 4,
    'bl_caster_l_wheel_joint': 4,
    'bl_caster_r_wheel_joint': 4,
    'br_caster_rotation_joint': 4,
    'br_caster_l_wheel_joint': 4,
    'br_caster_r_wheel_joint': 4,

    # Torso
    'torso_lift_joint': 5,

    # Head
    'head_pan_joint': 6,
    'head_tilt_joint': 6,

    # Laser scanner tilt motor
    'laser_tilt_mount_joint': 7,

    # Right arm
    'r_shoulder_pan_joint': 1,
    'r_shoulder_lift_joint': 1,
    'r_upper_arm_roll_joint': 1,
    'r_forearm_roll_joint': 1,
    'r_elbow_flex_joint': 1,
    'r_wrist_flex_joint': 1,
    'r_wrist_roll_joint': 1,

    # Right gripper
    'r_gripper_l_finger_joint': 3,
    'r_gripper_r_finger_joint': 3,
    'r_gripper_l_finger_tip_joint': 3,
    'r_gripper_r_finger_tip_joint': 3,

    # Left arm
    'l_shoulder_pan_joint': 0,
    'l_shoulder_lift_joint': 0,
    'l_upper_arm_roll_joint': 0,
    'l_forearm_roll_joint': 0,
    'l_elbow_flex_joint': 0,
    'l_wrist_flex_joint': 0,
    'l_wrist_roll_joint': 0,

    # Left gripper
    'l_gripper_l_finger_joint': 2,
    'l_gripper_r_finger_joint': 2,
    'l_gripper_l_finger_tip_joint': 2,
    'l_gripper_r_finger_tip_joint': 2
}

filename = "pr2_clean.ors"
for line in open(filename).readlines():
    if line.lower().startswith("joint "):
        match = re.match("^joint (\w*) \(", line)
        name = match.group(1)
        line = re.sub("}", "agent=%d }" % move_group[name], line)

    print line.strip()
