#include <ros/ros.h>
#include <string>
#include "particlefilter.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mcl3d_node");  
	
	// Particle filter instance
	std::string node_name = "mcl3d_node";
	ParticleFilter pf(node_name);
  
	// Process data at given rate
	ros::spin();

	return 0;
}




