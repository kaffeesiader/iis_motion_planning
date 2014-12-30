IISControl
==========

This package provides all the necessary infrastructure to interface our current control interface
with the requirements of moveit. It is based on the ros_control stack, that is part of the ROS hydro
distribution. 

After building the 'iis_control' package, you can launch the hardware adapter via
'roslaunch iis_control hardware_adapter.launch' for interfacing the real robot, or
'roslaunch iis_control hardware_adapter_simulation.launch' for interfacing the simulator.

When this node launches without errors everything is ready to interact with MoveIt.

Description
===========

The interface reads from our JointStates topics and publishes to our MoveJoints topics. It also provides
a controller_manager node that can load and run hardware controllers from the ROS control stack (for
example the FollowJointTrajectory Controller that can be used to execute MoveIt generated trajectories).

It is possible to create various configurations (for example one for the simulator and one for the
real robot) and switch between the configurations via a service call.

Switch between configurations via 'rosservice call /fake_controller_manager/load_config [config]'
where config can be 'sim' or 'real'. Topic names can be adjusted in the 'adapter_config.yaml' file

Launch the adapter using the 'hardware_adapter.launch' file for adapting the real robot, and
the 'hardware_adapter_sim.launch' file for adapting the simulator.

In order to work correctly the adapter needs the JointPositions of both arms. So it is necessary that both
arms are running and publishing joint state messages. So ensure that both arms are running and are in
control mode 10 or 30 if you use the real robot. Otherwise the hardware adapter will not work!

CAUTION!!!!
If the robot runs into an error and switches to another control mode, this adapter will keep on publishing
JointPosition messages to the arms. So if the arms move to another position in the meantime and you switch 
back to the control mode 10 or 30, the robot will instantly try to move very fast to the last commanded 
joint position. So please always shut down the adapter node in case of an error and relaunch it when everything
is fine again!!!