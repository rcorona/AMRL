#include <particle_filter/particle_filter.h>
#include <iostream> 
#include <random>
#include <math.h>
#include <algorithm> 

ParticleFilter::ParticleFilter() {
	//Mean actual movement in error model experiments. 
	double mean_movement = 1.88; 

	//Set forward error variance proportions. 
	translation_error.translation_error.variance_proportion = 5.2210535585492734e-05 / mean_movement;
	translation_error.rotation_error.variance_proportion = 0.011277530205766535 / mean_movement;

	//Set forward error mean per dimension.
	translation_error.translation_error.mean_proportion = 0.04622973302600004 / mean_movement;
	translation_error.rotation_error.mean_proportion = -0.73868934197905 / mean_movement;

	translation_error.rotation_error.variance_proportion = 0.011277530205 / mean_movement;
	translation_error.rotation_error.mean_proportion = -0.07 / mean_movement;

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

std::vector<Particle> ParticleFilter::get_particles() {
	return particles; 
}

void ParticleFilter::init(int num_particles) {
	//Resizes particle vector. 
	this->num_particles = num_particles; 
	particles.resize(num_particles); 

	for (int i = 0; i < num_particles; i++) {
		particles[i].pose.x = 0.0;
		particles[i].pose.y = 0.0; 
		particles[i].pose.theta = 0.0; 
	}
}

Pose ParticleFilter::get_odom_diff(Pose *odom_reading) {
	//Will contain difference in readings. 
	Pose diff; 

	//Computes differences. 
	diff.x = odom_reading->x - odom->x; 
	diff.y = odom_reading->y - odom->y;
	diff.theta = odom_reading->theta - odom->theta; 

	//Remembers reading for next iteration. 
	odom->x = odom_reading->x; 
	odom->y = odom_reading->y;
	odom->theta = odom_reading->theta; 

	return diff; 
}

void ParticleFilter::elapse_time(Pose *odom_reading) {
	//Initializes current reading if needed. 
	if (!odom) {
		odom = new Pose();

		//Copies values. 
		odom->x = odom_reading->x; 
		odom->y = odom_reading->y;
		odom->theta = odom_reading->theta; 
	}

	//Gets reported difference in odometry since last reading. 
	Pose odom_diff = get_odom_diff(odom_reading);

	//Compute translation gaussian based on reading. 
	compute_translation_gaussians(odom_reading); 

	//Elapses time for each particle given the reading.
	for (int i = 0; i < num_particles; i++)
		elapse_particle_time(&particles[i], &odom_diff); 
}



void ParticleFilter::elapse_particle_time(Particle *particle, Pose *reading) {
	//Estimates translation using error model and odometry estimate. 
	double odometry_trans = sqrt(pow(reading->x, 2) + pow(reading->y, 2));
	double translation_estimate = odometry_trans + translation_gauss(generator);

	//Gets rotation readings from odometry.
	double orientation_estimate = particle->pose.theta + rotation_gauss(generator);

	//Estimates x and y translations. 
	particle->pose.x += translation_estimate * cos(orientation_estimate);
	particle->pose.y += translation_estimate * sin(orientation_estimate);

	//Updates orientation. TODO Should second rotation error be added? 
	particle->pose.theta = orientation_estimate; 
}

void ParticleFilter::weigh_particles() {
	//Weighs each individual particle. 
	for (int i = 0; i < num_particles; i++)
		weigh_particle(&particles[i]); 
}

void ParticleFilter::weigh_particle(Particle *particle) {
	//TODO add sensor readings to actually weight particle.
	particle->weight = 1.0; 
}

void ParticleFilter::compute_translation_gaussians(Pose *reading) {
	//Gets estimated total translation. 
	double estimated_translation = sqrt(pow(reading->x, 2) + pow(reading->y, 2));

	//Computes mean based on forward error model. 
	double translation_mean = translation_error.translation_error.mean_proportion * estimated_translation;  
	double rotation_mean = translation_error.rotation_error.mean_proportion * estimated_translation; 

	//Computes variance and standard deviation based on forward error model.
	double translation_variance = translation_error.translation_error.variance_proportion * estimated_translation;
	double rotation_variance = translation_error.rotation_error.variance_proportion * estimated_translation; 

	double translation_std_dev = sqrt(translation_variance);  
	double rotation_std_dev = sqrt(rotation_variance);

	translation_gauss = std::normal_distribution<double>(translation_mean, translation_std_dev); 
	rotation_gauss = std::normal_distribution<double>(rotation_mean, rotation_std_dev); 
}

void ParticleFilter::resample_particles() {
	//Computes sum of weights in order to normalize. 
	double w_sum = 0.0;

	for (int i = 1; i < num_particles; i++)
		w_sum += particles[i].weight; 

	//Computes cumulative sum vector of normalized weights. 
	std::vector<double> c_sum(num_particles); 
	c_sum[0] = particles[0].weight / w_sum; 

	for (int i = 1; i < num_particles; i++)
		c_sum[i] = c_sum[i - 1] + particles[i].weight / w_sum;

	//Computes vector of random numbers and sorts them for resampling.  
	std::vector<double> samples = std::vector<double>(num_particles);
	std::uniform_real_distribution<double> distribution(0.0, 1.0); 

	for (int i = 0; i < num_particles; i++)
		samples[i] = distribution(generator);

	std::sort(samples.begin(), samples.end());

	//Determines which particle indeces were sampled. 
	int i = 0, j = 0; 
	std::vector<Particle> new_particles = std::vector<Particle>(num_particles); 

	while (i < num_particles) {
		//If true, then current particle's new state corresponds to the j'th particle's state. 
		if (samples[i] < c_sum[j]) {
			//Copies particle. 
			new_particles[i] = particles[j]; 
			i++; 
		}
		else
			j++; 
	}

	//Updates particles to reflect resampled values. 
	particles = new_particles; 
}
