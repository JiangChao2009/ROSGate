#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include "ros_gate.h"

int main(int argc, char* argv[]){
	ros::init(argc,argv,"ros_gate");
	ROSGate gate(8000);
	//gate.add_subscriber<std_msgs::Int16>("topic");
	gate.run();
}
