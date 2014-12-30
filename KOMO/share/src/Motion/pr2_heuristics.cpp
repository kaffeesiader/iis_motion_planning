/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#include "pr2_heuristics.h"
#include <Ors/ors.h>

arr pr2_zero_pose(){
  arr q = { 0.1, 0.999998, 0.500003, 0.999998, 1.5, -2, 0, 0.500003, 0, 0 };
  //{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, -1, 0.5, -1, -1.5, -2, 0, -0.5, 0, 0, 1, 0.5, 1, 1.5, -2, 0, 0.5, 0, 0 };
  return q;
}

arr pr2_reasonable_W(ors::KinematicWorld& world){
#if 0
  arr W = world.naturalQmetric(5.);
  ors::Joint *j = world.getJointByName("torso_lift_joint");
  if(j){
    CHECK(j->type == ors::JT_transX, "");
    W(j->qIndex) *= 10;
  }
  j = world.getJointByName("worldTranslationRotation");
  if(j){
    CHECK(j->type == ors::JT_transXYPhi, "");
    W(j->qIndex+0) *= 3;
    W(j->qIndex+1) *= 3;
//    W(j->qIndex+2) *= 10;
  }
#else
  arr W(world.getJointStateDimension());
  for(ors::Joint *j:world.joints){
    double h=j->H;
    for(uint k=0;k<j->qDim();k++) W(j->qIndex+k)=h;
  }
  cout <<W <<endl;
#endif
  return W;
}

uintA _get_shape_indices(ors::Body* b) {
  uintA idx;
  for(ors::Shape *s : b->shapes) {
    idx.append(s->index); 
  }
  return idx;
}

MT::Array<const char*> pr2_left_get_bodynames() {
  return { 
    "base_footprint",
    "torso_lift_link",
    "l_shoulder_pan_link",
    "l_shoulder_lift_link",
    "l_upper_arm_roll_link",
    "l_forearm_roll_link",
    "l_elbow_flex_link",
    "l_wrist_flex_link",
    "l_wrist_roll_link",
    "l_gripper_l_finger_link",
    "l_gripper_r_finger_link",
    "l_gripper_l_finger_tip_link",
    "l_gripper_r_finger_tip_link"
  };
}

MT::Array<const char*> pr2_full_get_bodynames() {
  return { 
    "base_footprint",
    "fl_caster_rotation_link",
    "fl_caster_l_wheel_link",
    "fl_caster_r_wheel_link",
    "fr_caster_rotation_link",
    "fr_caster_l_wheel_link",
    "fr_caster_r_wheel_link",
    "bl_caster_rotation_link",
    "bl_caster_l_wheel_link",
    "bl_caster_r_wheel_link",
    "br_caster_rotation_link",
    "br_caster_l_wheel_link",
    "br_caster_r_wheel_link",
    "torso_lift_link",
    "head_pan_link",
    "head_tilt_link",
    "laser_tilt_mount_link",
    "r_shoulder_pan_link",
    "r_shoulder_lift_link",
    "r_upper_arm_roll_link",
    "r_forearm_roll_link",
    "r_elbow_flex_link",
    "r_wrist_flex_link",
    "r_wrist_roll_link",
    "r_gripper_l_finger_link",
    "r_gripper_r_finger_link",
    "r_gripper_l_finger_tip_link",
    "r_gripper_r_finger_tip_link",
    "l_shoulder_pan_link",
    "l_shoulder_lift_link",
    "l_upper_arm_roll_link",
    "l_forearm_roll_link",
    "l_elbow_flex_link",
    "l_wrist_flex_link",
    "l_wrist_roll_link",
    "l_gripper_l_finger_link",
    "l_gripper_r_finger_link",
    "l_gripper_l_finger_tip_link",
    "l_gripper_r_finger_tip_link"
  };
    
}

uintA pr2_get_shapes(ors::KinematicWorld &G) {
  MT::Array<const char*> bodynames = pr2_left_get_bodynames();
  uintA shape_idx;
  for (const char* bodyname: bodynames) {
    shape_idx.append(_get_shape_indices(G.getBodyByName(bodyname)));
  }
  return shape_idx;
}
