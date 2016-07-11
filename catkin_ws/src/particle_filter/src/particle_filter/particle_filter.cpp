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
	this->forward_error.x_error.variance_proportion = 0.00111778179015 / 2.0;
	this->forward_error.y_error.variance_proportion = 0.00547198644888 / 2.0;

	//Set forward error mean per dimension.
	this->forward_error.x_error.mean_proportion = -0.132360749038 / 2.0;
	this->forward_error.y_error.mean_proportion = -0.730145944702 / 2.0;

	//Sets odometry reading to null. 
	this->odom = 0;
}

ParticleFilter::~ParticleFilter() {
	//Frees odometry reading memory if still allocated. 
	if (this->odom)
		delete this->odom; 
}

int ParticleFilter::get_num_particles() {
	return this->num_particles; 
}

particle_filter::Particle_vector ParticleFilter::get_particles() {
	return this->particles; 
}

void ParticleFilter::init(int num_particles) {
	//Resizes particle vector. 
	this->num_particles = num_particles; 
	this->particles.particles.resize(num_particles); 

	for (int i = 0; i < num_particles; i++) {
		this->particles.particles[i].pose.x = 0.0;
		this->particles.particles[i].pose.y = 0.0; 
		this->particles.particles[i].pose.theta = 0.0; 
	}
}

nav_msgs::Odometry ParticleFilter::get_odom_diff(nav_msgs::Odometry *odom) {
	//Will contain difference in readings. 
	nav_msgs::Odometry diff; 

	//Computes differences. 
	diff.pose.pose.position.x = odom->pose.pose.position.x - this->odom->pose.pose.position.x; 
	diff.pose.pose.position.y = odom->pose.pose.position.y - this->odom->pose.pose.position.y; 

	return diff; 
}

void ParticleFilter::elapse_time(nav_msgs::Odometry *odom) {
	//Initializes current reading if needed. 
	if (!this->odom) {
		this->odom = new nav_msgs::Odometry();

		//Copies values. 
		this->odom->pose.pose.position.x = odom->pose.pose.position.x; 
		this->odom->pose.pose.position.y = odom->pose.pose.position.y; 
	}


	//Gets reported difference in odometry since last reading. 
	nav_msgs::Odometry odom_diff = this->get_odom_diff(odom);

	//Elapses time for each particle given the reading.
	for (int i = 0; i < this->num_particles; i++)
		this->elapse_particle_time(&this->particles.particles[i], &odom_diff); 
}

void ParticleFilter::elapse_particle_time(particle_filter::Particle *particle, nav_msgs::Odometry *reading) {
	//Generates gaussians for particle's translation error.  
	std::normal_distribution<double> x_gauss = this->compute_x_gauss(reading); 
	std::normal_distribution<double> y_gauss = this->compute_y_gauss(reading);

	//Samples and sets new particle state.
	particle->pose.x += reading->pose.pose.position.x - x_gauss(this->generator);
	particle->pose.y += reading->pose.pose.position.y - y_gauss(this->generator); 
}

void ParticleFilter::weigh_particles() {
	//Weighs each individual particle. 
	for (int i = 0; i < this->num_particles; i++)
		this->weigh_particle(&this->particles.particles[i]); 
}

void ParticleFilter::weigh_particle(particle_filter::Particle *particle) {
	//TODO add sensor readings to actually weight particle.
	particle->weight = 1.0; 
}

std::normal_distribution<double> ParticleFilter::compute_x_gauss(nav_msgs::Odometry *reading) {
	//Computes mean based on forward error model. 
	double mean = this->forward_error.x_error.mean_proportion * reading->pose.pose.position.x; 

	//Computes variance and standard deviation based on forward error model.
	double variance = this->forward_error.x_error.variance_proportion * reading->pose.pose.position.x; 
	double std_dev = sqrt(variance);  

	return std::normal_distribution<double>(mean, std_dev); 
}

std::normal_distribution<double> ParticleFilter::compute_y_gauss(nav_msgs::Odometry *reading) {
	//Computes mean based on forward error model. 
	double mean = this->forward_error.y_error.mean_proportion * reading->pose.pose.position.y;

	//Computes variance and standard deviation based on forward error model.
	double variance = this->forward_error.x_error.variance_proportion * reading->pose.pose.position.y; 
	double std_dev = sqrt(variance);  

	return std::normal_distribution<double>(mean, std_dev); 
}

void ParticleFilter::resample_particles() {
	//Computes sum of weights in order to normalize. 
	double w_sum = 0.0;

	for (int i = 1; i < this->num_particles; i++)
		w_sum += this->particles.particles[i].weight; 

	//Computes cumulative sum vector of normalized weights. 
	std::vector<double> c_sum(this->num_particles); 
	c_sum[0] = this->particles.particles[0].weight / w_sum; 

	for (int i = 1; i < this->num_particles; i++)
		c_sum[i] = c_sum[i - 1] + this->particles.particles[i].weight / w_sum;

	//Computes vector of random numbers and sorts them for resampling.  
	std::vector<double> samples = std::vector<double>(this->num_particles);
	std::uniform_real_distribution<double> distribution(0.0, 1.0); 

	for (int i = 0; i < this->num_particles; i++)
		samples[i] = distribution(this->generator);

	std::sort(samples.begin(), samples.end());

	//Determines which particle indeces were sampled. 
	int i = 0, j = 0; 
	std::vector<particle_filter::Particle> new_particles = std::vector<particle_filter::Particle>(this->num_particles); 

	while (i < this->num_particles) {
		//If true, then current particle's new state corresponds to the j'th particle's state. 
		if (samples[i] < c_sum[j]) {
			//Copies particle. 
			new_particles[i] = this->particles.particles[j]; 
			i++; 
		}
		else
			j++; 
	}

	//Updates particles to reflect resampled values. 
	this->particles.particles = new_particles; 
}
