#include<ros/ros.h>
#include<iis_msgs/AddPrimitiveShape.h>
#include<std_msgs/String.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "komo_move");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Publisher pub_add = nh.advertise<iis_msgs::AddPrimitiveShape>("motion_control/scene/add_primitive_shape", 1, true);
	ros::Publisher pub_remove = nh.advertise<std_msgs::String>("motion_control/scene/remove_object", 1, true);

	ROS_INFO("Adding BOX primitive to planning scene");

	// define a BOX primitive
	iis_msgs::AddPrimitiveShape box_msg;
	box_msg.object_id = "box";
	box_msg.type = iis_msgs::AddPrimitiveShape::BOX;
	// set box dimensions
	box_msg.dimensions.push_back(0.1); // size_X
	box_msg.dimensions.push_back(0.1); // size_Y
	box_msg.dimensions.push_back(0.2); // size_Z
	// set position
	box_msg.pose.position.x = 0.1;
	box_msg.pose.position.y = 0.0;
	box_msg.pose.position.z = 0.19;
	// set orientation
	box_msg.pose.orientation.x = 0.0;
	box_msg.pose.orientation.y = 0.0;
	box_msg.pose.orientation.z = 0.0;
	box_msg.pose.orientation.w = 1.0;
	// enable or disable collision checking
	box_msg.disable_collision_checking = false;

	// send primitive to KOMO
	pub_add.publish(box_msg);

	sleep(1);

	// --------------------------------------------------

	ROS_INFO("Adding CYLINDER primitive to planning scene");

	// define a CYLINDER primitive
	iis_msgs::AddPrimitiveShape cyl_msg;
	cyl_msg.object_id = "cylinder";
	cyl_msg.type = iis_msgs::AddPrimitiveShape::CYLINDER;
	// set box dimensions
	cyl_msg.dimensions.push_back(0.20); // cylinder height
	cyl_msg.dimensions.push_back(0.05); // cylinder radius
	// set position
	cyl_msg.pose.position.x = 0.1;
	cyl_msg.pose.position.y = 0.4;
	cyl_msg.pose.position.z = 0.19;
	// set orientation
	cyl_msg.pose.orientation.x = 0.0;
	cyl_msg.pose.orientation.y = 0.0;
	cyl_msg.pose.orientation.z = 0.0;
	cyl_msg.pose.orientation.w = 1.0;
	// enable or disable collision checking
	cyl_msg.disable_collision_checking = false;

	// send primitive to KOMO
	pub_add.publish(cyl_msg);

	sleep(1);

	// --------------------------------------------------

	ROS_INFO("Adding SPHERE primitive to planning scene");

	// define a SPHERE primitive
	iis_msgs::AddPrimitiveShape sphere_msg;
	sphere_msg.object_id = "sphere";
	sphere_msg.type = iis_msgs::AddPrimitiveShape::SPHERE;
	// set box dimensions
	sphere_msg.dimensions.push_back(0.10); // sphere radius
	// set position
	sphere_msg.pose.position.x = 0.1;
	sphere_msg.pose.position.y = 0.8;
	sphere_msg.pose.position.z = 0.19;
	// set orientation
	sphere_msg.pose.orientation.x = 0.0;
	sphere_msg.pose.orientation.y = 0.0;
	sphere_msg.pose.orientation.z = 0.0;
	sphere_msg.pose.orientation.w = 1.0;
	// enable or disable collision checking
	sphere_msg.disable_collision_checking = false;

	// send primitive to KOMO
	pub_add.publish(sphere_msg);

	sleep(5);

	// ---------------------------------------------------

	std_msgs::String remove_msg;

	ROS_INFO("Removing BOX");
	// use the name of the object
	remove_msg.data = "box";
	pub_remove.publish(remove_msg);

	sleep(1);

	ROS_INFO("Removing CYLINDER");

	remove_msg.data = "cylinder";
	pub_remove.publish(remove_msg);

	sleep(1);

	ROS_INFO("Removing SPHERE");

	remove_msg.data = "sphere";
	pub_remove.publish(remove_msg);

	sleep(1);

	spinner.stop();

	return EXIT_SUCCESS;
}

