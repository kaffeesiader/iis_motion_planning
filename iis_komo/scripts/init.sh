#!/bin/bash

if [[ -z "$1" ]]
  then
    export NAMESPACE=simulation
  else
    export NAMESPACE=$1
fi

rostopic pub /${NAMESPACE}/right_arm/settings/switch_mode -1 std_msgs/Int32 10 &
rostopic pub /${NAMESPACE}/left_arm/settings/switch_mode -1 std_msgs/Int32 10 &

# rostopic pub /simulation/right_arm/joint_control/set_velocity_limit -1 std_msgs/Float32 0.1 & 
# rostopic pub /simulation/left_arm/joint_control/set_velocity_limit -1 std_msgs/Float32 0.1 & 

# table surface
rostopic pub /simulation/scene/AddPrimitiveShape planning_scene_plugin/AddPrimitiveShape -1 "object_id: 'surface' 
pose:
  position: {x: 0.171, y: 0.81, z: 0.045}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
disable_collision_checking: false
mass: 1.0
type: 1
dimensions: [1.2,2.3,0.09]" &

