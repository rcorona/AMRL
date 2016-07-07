#ifndef PARTICLEFILTER_PARTICLEFILTER_H
#define PARTICLEFILTER_PARTICLEFILTER_H

#include <ros/ros.h>
#include <particle_filter/Particle_vector.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <vector> 
#include <iostream>
#include <math.h>

struct Error {
	double variance_proportion; 
	double mean; 
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

	//Getters. 
	int get_num_particles();
	particle_filter::Particle_vector get_particles();

	void elapse_time(nav_msgs::Odometry *odom); 

private:
	std::default_random_engine engine; 

	//Error models. 
	ErrorModel forward_error;

	//Particle set variables. 
	particle_filter::Particle_vector particles; 
	int num_particles;

	//Keeps latest odometry readings for reference.
	nav_msgs::Odometry *odom;

	//Gets the difference between the current odom reading and the inputted one. 
	nav_msgs::Odometry get_odom_diff(nav_msgs::Odometry *odom);

	//Elapses time for particle. 
	void elapse_particle_time(particle_filter::Particle *particle, nav_msgs::Odometry *reading);

	//Methods for computing error gaussians. 
	std::normal_distribution<double> compute_x_gauss(nav_msgs::Odometry *reading);
	std::normal_distribution<double> compute_y_gauss(nav_msgs::Odometry *reading); 
};

#endif
