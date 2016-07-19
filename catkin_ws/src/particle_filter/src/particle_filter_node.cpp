#include <particle_filter/particle_filter.h>
#include <particle_filter/Particle_vector.h>
#include <particle_filter/Pose.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <eigen3/Eigen/Geometry>

//Odometry. 
nav_msgs::Odometry odom;
bool started = false;  

//Keeps GPS reading. 
particle_filter::Pose *gps_pose = 0; 

bool can_start() {
	return started && gps_pose; 
}

void gps_callback(particle_filter::Pose gps_reading) {
	//Initializes pose if first reading. 
	if (!gps_pose)
		gps_pose = new particle_filter::Pose(); 

	gps_pose->x = gps_reading.x;
	gps_pose->y = gps_reading.y; 
}

void odom_callback(nav_msgs::Odometry odom_reading) {
	if (!started)
		started = true; 

	odom = odom_reading;
}

void weigh_using_gps(Particle *particle, void **args) {
	double gps_x_variance = 2.172; 
	double gps_y_variance = 7.721; 

	double x_weight = 1 / (gps_x_variance + exp(std::abs(particle->pose.x - gps_pose->x))); 
	double y_weight = 1 / (gps_y_variance + exp(std::abs(particle->pose.y - gps_pose->y)));

	//Weighs particle using experimentally determined loss function. 
	particle->weight = x_weight * y_weight;  
}

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

particle_filter::Particle_vector read_in_particles(ParticleFilter *pf) {
	std::vector<Particle> particles = pf->get_particles(); 

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

	//Gets pose estimate for robot. 
	Pose pose_estimate = pf->get_pose_estimate();

	particle_vector.pose_estimate.x = pose_estimate.x;
	particle_vector.pose_estimate.y = pose_estimate.y;
	particle_vector.pose_estimate.theta = pose_estimate.theta; 

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
	ros::Rate loop_rate(1); 

	//Initialises the particle filter. 
	ParticleFilter pf; 
	pf.init(2000, &weigh_using_gps);

	//Subscribes to Odometry topic. 
	ros::Subscriber odom_sub = node.subscribe("jackal_velocity_controller/odom", 1000, odom_callback); 

	//Subscribes to GPS topic. 
	ros::Subscriber gps_sub = node.subscribe("gps_pos_meters", 1000, gps_callback); 

	//Waits for first odometry reading. 
	while (!can_start() && ros::ok())
		ros::spinOnce();

	while (ros::ok()) {
		//Publishes current set of particles and sleeps.
		pub.publish(read_in_particles(&pf)); 
		ros::spinOnce(); 
		loop_rate.sleep(); 

		//Elapses time for the filter. 
		Pose reading = odom_to_pose_reading(&odom);
		pf.elapse_time(&reading);

		//Weighs particles based on sensor readings. //TODO add sensor readings. 
		pf.weigh_particles(0);

		//Now resamples them. 
		pf.resample_particles();
	}

	return 0; 
}

