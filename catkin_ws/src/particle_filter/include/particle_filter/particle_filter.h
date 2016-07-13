#ifndef PARTICLEFILTER_PARTICLEFILTER_H
#define PARTICLEFILTER_PARTICLEFILTER_H

#include <ros/ros.h>
#include <particle_filter/Particle_vector.h>
#include <nav_msgs/Odometry.h>
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

class ParticleFilter {
public:
	ParticleFilter(); 
	~ParticleFilter(); 

	void init(int num_particles);

	//Getters. 
	int get_num_particles();
	particle_filter::Particle_vector get_particles();

	//Main particle filter algorithm methods. 
	void elapse_time(nav_msgs::Odometry *odom); 
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
	particle_filter::Particle_vector particles; 
	int num_particles;

	//Keeps latest odometry readings for reference.
	nav_msgs::Odometry *odom;

	//Gets the difference between the current odom reading and the inputted one. 
	nav_msgs::Odometry get_odom_diff(nav_msgs::Odometry *odom);
	
	//Gets rotation in radians from odometry reading. 
	double get_rotation_from_odom(nav_msgs::Odometry *reading); 

	//Elapses time for particle. 
	void elapse_particle_time(particle_filter::Particle *particle, nav_msgs::Odometry *reading);

	//Weights particles based on sensor readings. 
	void weigh_particle(particle_filter::Particle *particle); // TODO add sensor reading.  

	//Methods for computing error gaussians.
	void compute_translation_gauss(nav_msgs::Odometry *reading); 
};

#endif
