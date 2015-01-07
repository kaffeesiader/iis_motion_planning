##iis\_motion\_planning

This stack contains the required packages and libraries to integrate Marc Toussaint's KOMO motion optimization framework into the IIS-Lab robot setup. The KOMO sourcecode, along with some basic usage examples is located in the `KOMO/share` folder. General information about KOMO can be found in [this paper](http://arxiv.org/pdf/1407.0414v1.pdf). Please note that the framework is still under development and might be buggy.

The [`iis_komo`](https://github.com/kaffeesiader/iis_motion_planning/tree/master/iis_komo#iis_komo) package contains the ROS wrapper around the KOMO framework, along with the required .ors description of the IIS-Lab robot setup. The [`iis_control`](https://github.com/kaffeesiader/iis_motion_planning/tree/master/iis_control#iiscontrol) package provides the controller node, responsible for executing planned trajectories. Message and service definitions are located within the [`iis_msgs`](https://github.com/kaffeesiader/iis_motion_planning/tree/master/iis_msgs) package. The installation and build process is only tested on ROS hydro. As the [`iis_control`](https://github.com/kaffeesiader/iis_motion_planning/tree/master/iis_control#iiscontrol) package depends on the [`ros_control`](http://wiki.ros.org/ros_control) stack, it is not guaranteed that it will compile on ROS groovy.

##Installation and build

Clone the source code into the `src` folder of an existing CATKIN workspace:

	cd [MY_CATKIN_WS]/src
	git clone https://github.com/kaffeesiader/iis_motion_planning.git

**TODO:** *Move the sources into a git repository on the iis server. I cannot do that as I do not have the necessary privileges.*

The next step is to download and install all the required dependencies, which can be achieved by executing the corresponding bash script:

	cd [MY_CATKIN_WS]/src/iis_motion_planning
	./INSTALL_DEPENDENCIES.sh

Please be aware that executing this script requires administrator privileges. After that, the KOMO sourcecode has to be compiled first, because this build step is not included in the catkin build process: 

	make komo

The last step is to build the catkin workspace:

	make catkin


##Usage

Please have a look at the README file, provided in the [`iis_komo`](https://github.com/kaffeesiader/iis_motion_planning/tree/master/iis_komo#iis_komo) package.