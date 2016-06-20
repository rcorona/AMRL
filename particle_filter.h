/*
   Header file for a particle filter implementation. 
*/

#include <vector> 

/*
   Declarations for the Distribution base class. 
*/
class Distribution {
	:q
};

/*
   Declarations for the State class. 
*/
class State {
public:
	State(float x, float y); 
	~State(); 

private:
	//Location state variables. 
	float x; 
	float y; 

	// TODO float theta. 
};

/*
   Declarations for the Particle class. 
*/
class Particle {
public:
	Particle();
	~Particle(); 

private:
	//The location state of the particle. 
	State state; 

	//The weight currently assigned to this particle. 
	float weight; 
};

/*
   Declarations for the ParticleFilter class. 
*/
class ParticleFilter {
public:
	ParticleFilter();
	~ParticleFilter(); 

private:
	std::vector<Particle> particles; 
};
