#include <ros/ros.h>
#include <particle_filter/particle_filter.h>
#include <particle_filter/Particle_vector.h>
#include <iostream> 
#include <random>

ParticleFilter::ParticleFilter() {
	//Set forward error variance proportions. 
	this->forward_error.x_error.variance_proportion = 0.00111778179015;
	this->forward_error.y_error.variance_proportion = 0.00547198644888;

	//Set forward error mean per dimension.
	this->forward_error.x_error.mean = -0.132360749038;
	this->forward_error.y_error.mean = -0.730145944702;
}

ParticleFilter::~ParticleFilter() {
	// TODO
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
		this->particles.particles[i].pose.x = i;
		this->particles.particles[i].pose.y = i; 
		this->particles.particles[i].pose.theta = i; 
	}
}
