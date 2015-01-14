#iis_komo

This package contains the ROS wrapper around the KOMO framework, along with the required .ors description of the robot setup and some convenient launch scripts.

##Starting the KOMO node

The `iis_komo` node produces output files and depends on the configuration file `MT.cfg`. Therefore it is necessary to set the working directory to `iis_komo/tmp` before starting the node, otherwise it will not work. This happens automatically when using the provided launch file:

    roslaunch iis_komo iis_komo.launch
  
This statement launches the komo node for the simulator (default). To use it with the real robot, you can use:

    roslaunch iis_komo iis_komo.launch config_name:=real

This launch file also starts the controllers that are required to execute the generated trajectories on the simulated or real robot. The responsible node is located in the `iis_control` package. The simulator or real robot should be started and publishing to it's jointstates topics before launching the `iis_komo` node as the hardware controller expects the joinsstates topics to be present during startup. Otherwise you will get an error message on the console and trajectory execution is not possible.

##Planning trajectories

The planning service handles trajectory planning requests, based on the provided parameters. There are two modes available - planning in joint space and planning in Cartesian space.

**Service name:** `/[NAMESPACE]/motion_control/plan_trajectory`  
**Service type:** `iis_msgs/PlanTrajectory.srv`  

####Request params:

#####string planning_group
Currently either 'left_arm' or 'right_arm'. The plan is to create additional planning groups, for example left arm + gripper or both arms at once...

#####string eef_link
The name of the reference frame we want to plan for (only considered in CART_SPACE_MODE).
    
Currently available: 'left_eef', 'left_sdh_palm_link', 'left_sdh_grasp_link', 'left_sdh_tip_link', 'right_eef', 'right_sdh_...'
      
Those reference frames are defined within the file `iis_komo/data/iis_robot.kvg`. Additional frames can easily be added, if required.

#####int32 mode
The planning mode - can be either 

* CART_SPACE_MODE = 0
* JOINT_SPACE_MODE = 1

#####sensor_msgs/Jointstate [OPTIONAL]
An alternating start state. If no start state is provided, the KOMO node will use the robot's current state as read from the robot's `joint_control/get_state` topics.

#####float64[] target
An array of values, describing the desired goal. The target specification is interpreted as follows:
   
CART_SPACE_MODE:

* 6 values -> [x,y,z,roll,pitch,yaw]
* 7 values -> [x,y,z,qx,qy,qz,qw]
* 3 values -> [x,y,z]  (position only)
  
JOINT_SPACE_MODE:

* 7 values -> one value for each arm joint

#####geometry_msgs/Vector3 position_tolerance
A vector, specifying the allowed position tolerances, to consider a planning attempt to be successful. If the desired position tolerances are not satisfied but the result is not invalid because of a collision or joint limit violation, the resulting solution will be marked as APPROXIMATE in the response. This parameter is only considered in CART_SPACE_MODE.

#####geometry_msgs/Vector3 angular_tolerance
A vector, specifying the allowed angular tolerances, to consider a planning attempt to be successful. If the desired position tolerances are not satisfied but the result is not invalid because of a collision or joint limit violation, the resulting solution will be marked as APPROXIMATE in the response. This parameter is only considered in CART_SPACE_MODE.

#####int32 axes_to_align
This parameter allows to select, which axes of the end effector reference frame need to be alligned with the corresponding axes of the target reference frame. The value is bit coded and can be combined like
  
        axes_to_align = X_AXIS | Y_AXIS | Z_AXIS;
  
where X_AXIS = 1, Y_AXIS = 2 and Z_AXIS = 4. For example if only the z-axis of the end effector reference frame should be aligned with the z-axis of the target reference frame it could be done like
  
        axes_to_align = 4;

#####bool allow_support_surface_contact
True means the robot is allowed to touch the sponge on the table. Therefore it will be removed from collision checking. HANDLE WITH CARE!!!
        
####Response params:

#####string planning_group
The planning group, the planning request was done for.

#####trajectory_msgs/JointTrajectory trajectory_msgs
The resulting trajectory (waypoints only contain joint positions, no time parameterization!).

#####geometry_msgs/Vector3 linear_error
The calculated error in position.

#####geometry_msgs/Vector3 angular_error
The calculated angular error (angle between desired and actual orientation for each axis in radiant).

#####geometry_msgs/Pose goal_pose
The calculated goal pose.

#####int32 status
The overall status(i.e. planning was successful or not).
Possible values:

* SUCCESS     = 1 -> Planning was successful and goal tolerances were met
* APPROXIMATE = 2 -> Resulting goal pose lies not within specified goal tolerances
* FAILED      = 0 -> Planning failed because of joint limit violation or detected collision

#####string error
Contains an error message in case that planning failed or solution is only approximate.

#####float64 planning_time
The measured planning time in seconds

##Executing planned trajectories

The execution service can be used to execute previously planned trajectories. The desired execution speed can be specified as a percentage of maximum allowed speed.

**Service name:** `/[NAMESPACE]/motion_control/execute_trajectory`  
**Service type:** `iis_msgs/ExecuteTrajectory.srv`  

####Request params:

#####string planning_group
The name of the planning group. Can be either 'left_arm' or 'right_arm'.

#####trajectory_msgs/JointTrajectory trajectory_msgs   
The trajectory to execute on the given planning group. This is usually the outcome of a previous planning request. The given trajectory needs only joint positions for each waypoint, as the time-parameterization will be computed, based on the specified velocity_factor.
    
#####float64 velocity_factor
A value between 0 and 1 specifying the percentage of maximum allowed velocity.

####Response params:

#####bool result
Specifies whether execution was successful or not.

The folder `iis_komo/test` contains different examples for using those services. Please have a look at the section [Usage examples](#usage). The following ROS topics can be used to manipulate the planning scene.

##Adding primitive shapes to the planning scene

**Topic name:**   `/[NAMESPACE]/motion_control/scene/add_primitive_shape`  
**Message type:** `iis_msgs/AddPrimitiveShape`  
    
Adds a primitive shape to the planning scene. The shape will be inserted at the given pose, with given name. The message type is the same that is used by the simulator to add shapes to the simulation scene, but the message definition was moved into the `iis_msgs` package to avoid unnecessary dependecies. The message definitions of the simulator could also be moved into that package in future. The `mass` parameter can be set but it is ignored currently. The origin of the shape reference frame is always located at the center of the shape. The `type` parameter specifies the type of the required shape. Supported shape types are BOX (type: 1), SPHERE (type: 2) and CYLINDER (type: 3).

The `dimensions` parameter is of type `float64[]` and specifies the required dimensions, based on given shape type:
  
      BOX        - dimensions: [SIZE_X, SIZE_Y, SIZE_Z]
      SPHERE     - dimensions: [RADIUS]
      CYLINDER   - dimensions: [CYLINDER_HEIGHT, CYLINDER_RADIUS]

The `disable_collision_checking` parameter allows to exclude the new shape from collision checking.
    
Example call (creates a box named 'my_box' with given dimensions on given position):

    rostopic pub -1 /simulation/motion_control/scene/add_primitive_shape iis_msgs/AddPrimitiveShape "object_id: 'my_box'
        pose:
          position: {x: 0.0, y: 0.0, z: 0.1}
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        disable_collision_checking: true
        type: 1
        dimensions: [0.2, 0.2, 0.2]" 
    

Example call (creates a cylinder named 'my_cylinder' with given dimensions on given position):

    rostopic pub -1 /simulation/motion_control/scene/add_primitive_shape iis_msgs/AddPrimitiveShape "object_id: 'my_cylinder'
        pose:
          position: {x: 0.0, y: 0.0, z: 0.1}
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        disable_collision_checking: false
        type: 3
        dimensions: [0.2, 0.05]" 

##Removing objects from the planning scene

**Topic name:**   `/[NAMESPACE]/motion_control/scene/remove_object`  
**Message type:** `std_msgs/String`  

Removes the object with given name from the planning scene.

Example call:

    rostopic pub -1 /simulation/motion_control/scene/remove_object std_msgs/String "data: 'my_object'"

##Usage examples<a name="usage"></a>

The `iis_komo` node provides two basic services for planning and executing trajectories. Please have a look into the sample files `test/move_cart.cpp` and `test/move_joint.cpp` for detailed usage examples of those services! The samples can be run as follows:

Start the komo node in the desired namespace (either 'simulation' or 'real')

    roslaunch iis_komo iis_komo.launch config_name:=[NAMESPACE]

Set the `ROS_NAMESPACE` environment variable in the terminal you use to run the test code to the desired namespace:

    export ROS_NAMESPACE=[NAMESPACE]

Run the sample code:

    rosrun iis_komo move_cart [MOVE_GROUP] [EEF_LINK] posX posY posZ

for position only targets or

    rosrun iis_komo move_cart [MOVE_GROUP] [EEF_LINK] posX posY posZ ROLL PITCH YAW
or

    rosrun iis_komo move_cart [MOVE_GROUP] [EEF_LINK] posX posY posZ quatX quatY quatZ quatW

for Cartesian space targets or

    rosrun iis_komo move_joint [MOVE_GROUP] JNT0 JNT1 JNT2 JNT3 JNT4 JNT5 JNT6

for joint space targets.

MOVE_GROUP can be either `left_arm` or `right_arm`
EEF_LINK can be `left_arm_7_link`, `left_sdh_grasp_link`, `left_sdh_tip_link`, `left_sdh_palm_link`, `right_arm_7_link`, `right_sdh...`

##Notes

  * The results may differ on different computers. So it could be necessary to play with the precision parameters, defined in the ''iis_komo/config/MT.cfg'' configuration file. Try to raise or lower the settings for position and alignment precision until you receive the desired results, but beware of possible side effects.
  * KOMO did compile on one of the PC in the robot lab properly, but the `iis_komo` package did not, because of a linker error. I solved the problem by copying the `libOptim.so` library file from my computer to the `KOMO/share/lib` folder of the lab PC. However, I wasn't able to figure out why that happened...

##Open questions

  * How to keep the robot in "good" configurations to improve the overall success rate?
  * How to improve and maximize reachability? Tests showed that the success rate for Cartesian space goals lies only around 60 to 70 percent. Planning always fails because the planner runs into joint limits.

##Possible improvements

  * Decouple trajectory execution from planning. Currently, the komo_node does both but execution functionality should be moved to `iis_control` package.
  * Allow to specify maximum velocities in configuration file (currently hardcoded within `iis_robot.h` header file).
  * Provide a way to disable collision checking for specific pairs of shapes 
  * Provide a way to attach objects to the grippers (as shown in `KOMO/share/examples/KOMO/ikea_chair` example).
  * Test adding/remove collision object functionality in depth (especially removed objects could result in an error during subsequent planning requests...)

