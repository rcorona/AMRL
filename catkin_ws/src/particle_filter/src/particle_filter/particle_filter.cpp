#include <ros/ros.h>
#include <particle_filter/particle_filter.h>
#include <particle_filter/Particle_vector.h>
#include <nav_msgs/Odometry.h>
#include <iostream> 
#include <random>
#include <math.h>

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
	particle->pose.x += reading->pose.pose.position.x - x_gauss(this->engine);
	particle->pose.y += reading->pose.pose.position.y - y_gauss(this->engine); 
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
