#include <ros/ros.h>
#include <particle_filter/particle_filter.h>
#include <particle_filter/Particle_vector.h>
#include <nav_msgs/Odometry.h>
#include <iostream> 
#include <random>
#include <math.h>
#include <algorithm> 

ParticleFilter::ParticleFilter() {
	//Set forward error variance proportions. 
	translation_error.translation_error.variance_proportion = 0.00111778179015 / 2.0;
	translation_error.rotation_error.variance_proportion = 0.00547198644888 / 2.0;

	//Set forward error mean per dimension.
	translation_error.translation_error.mean_proportion = -0.132360749038 / 2.0;
	translation_error.rotation_error.mean_proportion = -0.730145944702 / 2.0;

	//Sets odometry reading to null. 
	odom = 0;
}

ParticleFilter::~ParticleFilter() {
	//Frees odometry reading memory if still allocated. 
	if (odom)
		delete odom; 
}

int ParticleFilter::get_num_particles() {
	return num_particles; 
}

particle_filter::Particle_vector ParticleFilter::get_particles() {
	return particles; 
}

void ParticleFilter::init(int num_particles) {
	//Resizes particle vector. 
	this->num_particles = num_particles; 
	particles.particles.resize(num_particles); 

	for (int i = 0; i < num_particles; i++) {
		particles.particles[i].pose.x = 0.0;
		particles.particles[i].pose.y = 0.0; 
		particles.particles[i].pose.theta = 0.0; 
	}
}

nav_msgs::Odometry ParticleFilter::get_odom_diff(nav_msgs::Odometry *odom_reading) {
	//Will contain difference in readings. 
	nav_msgs::Odometry diff; 

	//Computes differences. 
	diff.pose.pose.position.x = odom_reading->pose.pose.position.x - odom->pose.pose.position.x; 
	diff.pose.pose.position.y = odom_reading->pose.pose.position.y - odom->pose.pose.position.y;

	//Remembers reading for next iteration. 
	odom->pose.pose.position.x = odom_reading->pose.pose.position.x; 
	odom->pose.pose.position.y = odom_reading->pose.pose.position.y; 

	return diff; 
}

void ParticleFilter::elapse_time(nav_msgs::Odometry *odom_reading) {
	//Initializes current reading if needed. 
	if (!odom) {
		odom = new nav_msgs::Odometry();

		//Copies values. 
		odom->pose.pose.position.x = odom_reading->pose.pose.position.x; 
		odom->pose.pose.position.y = odom_reading->pose.pose.position.y; 
	}

	//Gets reported difference in odometry since last reading. 
	nav_msgs::Odometry odom_diff = get_odom_diff(odom_reading);

	//Compute translation gaussian based on reading. 
	compute_translation_gauss(odom_reading); 

	//Elapses time for each particle given the reading.
	for (int i = 0; i < num_particles; i++)
		elapse_particle_time(&particles.particles[i], &odom_diff); 
}

double ParticleFilter::get_rotation_from_odom(nav_msgs::Odometry *reading) {
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

void ParticleFilter::elapse_particle_time(particle_filter::Particle *particle, nav_msgs::Odometry *reading) {
	//Estimates translation using error model and odometry estimate. 
	double odometry_trans = sqrt(pow(reading->pose.pose.position.x, 2) + pow(reading->pose.pose.position.y, 2));
	double translation_estimate = odometry_trans + translation_gauss(generator);

	//Gets rotation readings from odometry.
	double rotation_reading = get_rotation_from_odom(reading);
	double rotation_estimate = rotation_reading; //TODO add error. 
}

void ParticleFilter::weigh_particles() {
	//Weighs each individual particle. 
	for (int i = 0; i < num_particles; i++)
		weigh_particle(&particles.particles[i]); 
}

void ParticleFilter::weigh_particle(particle_filter::Particle *particle) {
	//TODO add sensor readings to actually weight particle.
	particle->weight = 1.0; 
}

void ParticleFilter::compute_translation_gauss(nav_msgs::Odometry *reading) {
	//Gets estimated total translation. 
	double estimated_translation = sqrt(pow(reading->pose.pose.position.x, 2) + pow(reading->pose.pose.position.y, 2));

	//Computes mean based on forward error model. 
	double mean = translation_error.translation_error.mean_proportion * estimated_translation;  

	//Computes variance and standard deviation based on forward error model.
	double variance = translation_error.translation_error.variance_proportion * estimated_translation; 
	double std_dev = sqrt(variance);  

	translation_gauss = std::normal_distribution<double>(mean, std_dev); 
}

void ParticleFilter::resample_particles() {
	//Computes sum of weights in order to normalize. 
	double w_sum = 0.0;

	for (int i = 1; i < num_particles; i++)
		w_sum += particles.particles[i].weight; 

	//Computes cumulative sum vector of normalized weights. 
	std::vector<double> c_sum(num_particles); 
	c_sum[0] = particles.particles[0].weight / w_sum; 

	for (int i = 1; i < num_particles; i++)
		c_sum[i] = c_sum[i - 1] + particles.particles[i].weight / w_sum;

	//Computes vector of random numbers and sorts them for resampling.  
	std::vector<double> samples = std::vector<double>(num_particles);
	std::uniform_real_distribution<double> distribution(0.0, 1.0); 

	for (int i = 0; i < num_particles; i++)
		samples[i] = distribution(generator);

	std::sort(samples.begin(), samples.end());

	//Determines which particle indeces were sampled. 
	int i = 0, j = 0; 
	std::vector<particle_filter::Particle> new_particles = std::vector<particle_filter::Particle>(num_particles); 

	while (i < num_particles) {
		//If true, then current particle's new state corresponds to the j'th particle's state. 
		if (samples[i] < c_sum[j]) {
			//Copies particle. 
			new_particles[i] = particles.particles[j]; 
			i++; 
		}
		else
			j++; 
	}

	//Updates particles to reflect resampled values. 
	particles.particles = new_particles; 
}
