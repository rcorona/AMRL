#!/usr/bin/python

"""
Used to visualize different types of Jackal data
from bag files in a variety of ways. 
"""

import rospy
import rosbag
import sys
from math import sqrt
import os
from overlay_to_map import read_in_overlay_specs 
import matplotlib.pyplot as plt
from geopy.distance import VincentyDistance
from scipy.misc import imread

class Visualizer: 
    def __init__(self, bag_file, map_specs):
        #Opens bag file. 
        self.bag_file = rosbag.Bag(bag_file, 'r')

        #Determines coordinates of corner of map image. 
        map_img_name, self.BR, self.TL = read_in_overlay_specs(map_specs)
        self.width = float(VincentyDistance((self.BR[0], self.TL[1]), (self.BR[0], self.BR[1])).meters)
        self.height = float(VincentyDistance((self.BR[0], self.TL[1]), (self.TL[0], self.TL[1])).meters)

        #Loads map image onto plot using corner coordinates. 
        self.figure = plt.figure()
        self.map_img_file = imread(map_img_name)
        plt.imshow(self.map_img_file, zorder=0, extent=[0.0, self.width, 0.0, self.height])

    def gps_to_meters(self, latitude, longitude):
        #Translates coordinate point based on formula below. 
        h = float(VincentyDistance((self.BR[0], self.TL[1]), (latitude, longitude)).meters)
        e = float(VincentyDistance((self.BR[0], self.BR[1]), (latitude, longitude)).meters)

        x = ((h ** 2) + (self.width ** 2) - (e ** 2)) / (2 * self.width)
        y = sqrt((h ** 2) - (x ** 2))

        return [x, y]

    def plot_gps(self, topic_data):
        x_points = []
        y_points = []

        #Processes each message.  
        for msg in topic_data['/navsat/fix']:
            x, y = self.gps_to_meters(msg.latitude, msg.longitude)

            x_points.append(x)
            y_points.append(y)

        #Plots points. 
        plt.plot(x_points, y_points, 'ro')
        plt.show()

    def plot_odom(self, topic_data):
        x_points = []
        y_points = []

        #Gets difference in order to synchronize coordinate planes.
        start_gps = topic_data['/navsat/fix'][0]
        x_0, y_0 = self.gps_to_meters(start_gps.latitude, start_gps.longitude)

        start_odom = topic_data['/odometry/filtered'][0]
        dx = x_0 - start_odom.pose.pose.position.x
        dy = y_0 - start_odom.pose.pose.position.y

        #Processes each message. 
        for msg in topic_data['/odometry/filtered']:
            x = msg.pose.pose.position.x + dx
            y = msg.pose.pose.position.y + dy

            x_points.append(x)
            y_points.append(y)

        #Plots points. 
        plt.plot(x_points, y_points, 'ro')
        plt.show()

    def plot_data(self, topic, topic_data):
        if topic == '/navsat/fix':
            self.plot_gps(topic_data)
        elif topic == '/odometry/filtered':
            self.plot_odom(topic_data)

    def plot_topic(self, topic_name):
        #Compiles all data for relevant topic. 
        topic_data = {'/navsat/fix': [], 
                '/odometry/filtered': []}

        for topic, msg, t in self.bag_file.read_messages():
            #Checks if msg topic is to be visualized.
            if topic in topic_data:
                topic_data[topic].append(msg)
    
        #Plots data. 
        self.plot_data(topic_name, topic_data)

if __name__ == '__main__':
    visualizer = Visualizer(sys.argv[1], sys.argv[2])
    
    visualizer.plot_topic(sys.argv[3])
            
