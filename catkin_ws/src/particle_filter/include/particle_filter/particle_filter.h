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

	void init(int num_particles, void (*weighing_func)(Particle *, void **));

	//Getters. 
	int get_num_particles();
	std::vector<Particle> get_particles();
	Pose get_pose_estimate(); 

	//Main particle filter algorithm methods. 
	void elapse_time(Pose *odom_reading); 
	void weigh_particles(void **args); // TODO add sensor readings.  
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

	//Keeps estimate of robot's pose. 
	Pose pose_estimate;
	void estimate_current_position(); 

	//Keeps latest odometry readings for reference.
	Pose *odom;

	//Particle weighing function pointer, allows for modularity of filter.  
	void (*weighing_function)(Particle *, void **); 

	//Gets the difference between the current odom reading and the inputted one. 
	Pose get_odom_diff(Pose *odom_reading);
	
	//Elapses time for particle. 
	void elapse_particle_time(Particle *particle, Pose *reading);

	//Methods for computing error gaussians.
	void compute_translation_gaussians(Pose *reading); 
};

#endif
