#ifndef PARTICLEFILTER_PARTICLEFILTER_H
#define PARTICLEFILTER_PARTICLEFILTER_H

#include <ros/ros.h>
#include <particle_filter/Particle_vector.h>
#include <random>
#include <vector> 
#include <iostream>

struct Error {
	double variance_proportion; 
	double mean; 
};

struct GaussianVector {
	std::normal_distribution<double> x_gaussian; 
	std::normal_distribution<double> y_gaussian; 
	std::normal_distribution<double> theta_gaussian; 
};

struct ErrorModel {
	Error x_error; 
	Error y_error; 
	Error theta_error;
};

class ParticleFilter {
public:
	ParticleFilter(); 
	~ParticleFilter(); 

	void init(int num_particles);

	int get_num_particles();
	particle_filter::Particle_vector get_particles(); 
private:
	std::default_random_engine engine; 

	//Error models. 
	ErrorModel forward_error;

	particle_filter::Particle_vector particles; 
	int num_particles; 
};

#endif
