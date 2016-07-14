#ifndef PARTICLEFILTER_PARTICLEFILTER_H
#define PARTICLEFILTER_PARTICLEFILTER_H

#include <random>
#include <vector> 
#include <iostream>
#include <math.h>
#include <algorithm>
#include <eigen3/Eigen/Geometry>

struct Error {
	double variance_proportion; 
	double mean_proportion; 
};

struct ErrorModel {
	Error translation_error; 
	Error rotation_error; 
};

struct Pose {
	double x; 
	double y; 
	double theta; 
};

struct Particle {
	Pose pose;  
	double weight; 
};

class ParticleFilter {
public:
	ParticleFilter(); 
	~ParticleFilter(); 

	void init(int num_particles);

	//Getters. 
	int get_num_particles();
	std::vector<Particle> get_particles();

	//Main particle filter algorithm methods. 
	void elapse_time(Pose *odom_reading); 
	void weigh_particles(); // TODO add sensor readings.  
	void resample_particles(); 

private:
	//Random number generator. 
	std::default_random_engine generator; 
	
	//Gaussians used for sampling. 
	std::normal_distribution<double> translation_gauss; 
	std::normal_distribution<double> rotation_gauss; 

	//Error models. 
	ErrorModel translation_error;

	//Particle set variables. 
	std::vector<Particle> particles; 
	int num_particles;

	//Keeps latest odometry readings for reference.
	Pose *odom;

	//Gets the difference between the current odom reading and the inputted one. 
	Pose get_odom_diff(Pose *odom_reading);
	
	//Elapses time for particle. 
	void elapse_particle_time(Particle *particle, Pose *reading);

	//Weights particles based on sensor readings. 
	void weigh_particle(Particle *particle); // TODO add sensor reading.  

	//Methods for computing error gaussians.
	void compute_translation_gauss(Pose *reading); 
};

#endif
