#!/usr/bin/python 

import turtle
import rospy
from particle_filter.msg import Particle_vector

class ParticleVisualizer:
    def __init__(self):
        #Creates display for visualization. 
        self.window = turtle.Screen()
        turtle.setworldcoordinates(-50, -50, 50, 50)

        #Initializes ROS and subscribes to particle filter. 
        rospy.init_node('particle_visualizer', anonymous=True)
        rospy.Subscriber('particle_filter', Particle_vector, self.update) 
        #Initializes list of particles. 
        self.particles = []

    def init_particles(self, particle_vector): 
        #Clears particles. 
        self.particles = []

        #Creates a turtle object for each particle. 
        for i in range(len(particle_vector)):
            #Creates turtle and appends it. 
            new_turtle = turtle.Turtle()
            new_turtle.hideturtle()
            new_turtle.penup()

            self.particles.append(new_turtle)

    def update_particles(self, particle_vector):
        #Updates each particle given the new reading. 
        for i in range(len(particle_vector)):
            #Gets readings. 
            x = particle_vector[i].pose.x
            y = particle_vector[i].pose.y
            theta = particle_vector[i].pose.theta

            #Updates values based on readings. 
            self.particles[i].setx(x)
            self.particles[i].sety(y)

            #Re-draws turtle.
            self.particles[i].dot(5, 'red')

    def update(self, particle_vector):
        #Re-initialize particles if number has changed. 
        if len(self.particles) != len(particle_vector.particles):
            self.init_particles(particle_vector.particles)

        #Updates particle positions. 
        self.update_particles(particle_vector.particles)

    def run(self):
        turtle.mainloop()

if __name__ == "__main__":
    pv = ParticleVisualizer()

    pv.run()
