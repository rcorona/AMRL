#!/usr/bin/python

import sys
import matplotlib.pyplot as plt
import rospy 
import rosbag

colors = ['ro', 'bo', 'go', 'co', 'yo']

def plot_particles(rosbag_name):
    bag = rosbag.Bag(rosbag_name, 'r')
    counter = 0

    for topic, msg, t in bag.read_messages():
        if topic == '/particle_filter':
            if counter <= 10: 
                x_points = []
                y_points = []

                for i in range(len(msg.particles)):
                    x_points.append(msg.particles[i].pose.x)
                    y_points.append(msg.particles[i].pose.y)

                plt.plot(x_points, y_points, colors[counter % 5])

            counter += 1
    
    plt.gca().set_xlim([-10, 100])
    plt.gca().set_ylim([-15, 60])
    plt.show()
             

plot_particles(sys.argv[1])
