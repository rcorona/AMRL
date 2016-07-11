#include <particle_filter/particle_filter.h>
#include <particle_filter/Particle_vector.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char **argv) {
	//Initialize particle filter node. 
	ros::init(argc, argv, "particle_filter"); 
	ros::NodeHandle node; 

	//Initialize publisher and rate. 
	ros::Publisher pub = node.advertise<particle_filter::Particle_vector>("particle_filter", 1000); 
	ros::Rate loop_rate(1); 

	//Initialises the particle filter. 
	ParticleFilter pf; 
	pf.init(50);

	int count = 0;

	//Sets initial test odometry reading. 
	nav_msgs::Odometry odom;
	odom.pose.pose.position.x = 0.0; 
	odom.pose.pose.position.y = 0.0; 

	while (ros::ok()) {
		//Publishes current set of particles and sleeps. 
		pub.publish(pf.get_particles()); 
		ros::spinOnce(); 
		loop_rate.sleep(); 

		count++;

		//Elapses time for the filter. 
		odom.pose.pose.position.x += 2.0; 
		odom.pose.pose.position.y += 2.0; 
		pf.elapse_time(&odom);

		//Weighs particles based on sensor readings. //TODO add sensor readings. 
		pf.weigh_particles(); 
	}

	return 0; 
}

