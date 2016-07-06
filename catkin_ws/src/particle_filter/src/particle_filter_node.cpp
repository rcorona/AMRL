#include <particle_filter/particle_filter.h>
#include <particle_filter/Particle_vector.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
	//Initialize particle filter node. 
	ros::init(argc, argv, "particle_filter"); 
	ros::NodeHandle node; 

	//Initialize publisher and rate. 
	ros::Publisher pub = node.advertise<particle_filter::Particle_vector>("particle_filter", 1000); 
	ros::Rate loop_rate(10); 

	//Initialises the particle filter. 
	ParticleFilter pf; 
	pf.init(50);
	
	int count = 0;

	while (ros::ok()) {
		pub.publish(pf.get_particles()); 
		ros::spinOnce(); 
		loop_rate.sleep(); 

		count++; 
	}

	return 0; 
}

