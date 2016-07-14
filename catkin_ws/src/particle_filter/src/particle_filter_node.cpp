#include <particle_filter/particle_filter.h>
#include <particle_filter/Particle_vector.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Geometry>

double get_rotation_from_odom(nav_msgs::Odometry *reading) {
	//Gets quaternion values.
	double x = reading->pose.pose.orientation.x; 
	double y = reading->pose.pose.orientation.y;
	double z = reading->pose.pose.orientation.z;
	double w = reading->pose.pose.orientation.w; 

	Eigen::Quaternion<double> quaternion(x, y, z, w); 

	//Gets rotation matrix from quaternion.
	Eigen::Matrix<double, 3, 3> rotation_matrix = quaternion.normalized().toRotationMatrix(); 

	//Gets Euler angles from the matrix. 
	Eigen::Matrix<double, 3, 1> angles = rotation_matrix.eulerAngles(2, 1, 0); 

	//Returns yaw (i.e. rotation in heading).  
	return angles(2); 
}

particle_filter::Particle_vector read_in_particles(std::vector<Particle> particles) {
	//Creates particle vector with necessary number of particles. 
	particle_filter::Particle_vector particle_vector; 
	particle_vector.particles = std::vector<particle_filter::Particle>(particles.size());

	//Copies each particle onto message. 
	for (int i = 0; i < particles.size(); i++) {
		//Copies pose. 
		particle_vector.particles[i].pose.x = particles[i].pose.x; 
		particle_vector.particles[i].pose.y = particles[i].pose.y; 
		particle_vector.particles[i].pose.theta = particles[i].pose.theta;

		//Copies weight. 
		particle_vector.particles[i].weight = particles[i].weight; 
	}

	return particle_vector; 
}

Pose odom_to_pose_reading(nav_msgs::Odometry *odom) {
	Pose reading; 

	//Gets translational reading. 
	reading.x = odom->pose.pose.position.x; 
	reading.y = odom->pose.pose.position.y; 

	//Gets rotational reading. 
	reading.theta = get_rotation_from_odom(odom); 

	return reading; 
}

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

	//Sets initial test odometry reading. 
	nav_msgs::Odometry odom;
	odom.pose.pose.position.x = 0.0; 
	odom.pose.pose.position.y = 0.0; 

	while (ros::ok()) {
		//Publishes current set of particles and sleeps.
		pub.publish(read_in_particles(pf.get_particles())); 
		ros::spinOnce(); 
		loop_rate.sleep(); 

		//Elapses time for the filter. 
		odom.pose.pose.position.x += 2.0; 
		odom.pose.pose.position.y += 2.0; 
		Pose reading = odom_to_pose_reading(&odom); 
		pf.elapse_time(&reading);

		//Weighs particles based on sensor readings. //TODO add sensor readings. 
		pf.weigh_particles();

		//Now resamples them. 
		pf.resample_particles(); 
	}

	return 0; 
}

