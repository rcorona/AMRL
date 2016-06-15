#!/usr/bin/python

"""
This script overlays the gps points held within a label
file (as prepared using pull_images_and_labels.py) or a csv file
for a jackal run onto a google map.

Usage: ./overlay_to_map.py [map_specs_file] [coordinate file]

Parameters: 
    map_specs_file - A text file containing the specifications
                     for the map overlay in the following format:

                     map_img_name
                     bottom_right_corner_latitude;bottom_right_corner_longitude
                     top_left_corner_latitude;top_left_corner_longitude

    coordinate file - The file to read the coordinates from. This will most
                      likely be the labels.txt file generated using
                      pull_images_and_labels.py but may also be 
                      a .csv file. 


Author: Rodolfo Corona, rcorona@utexas.edu
"""

import rospy
import rosbag
import sys
from geopy.distance import VincentyDistance
import matplotlib.pyplot as plt
from scipy.misc import imread
import matplotlib.cbook as cbook

"""
Reads and returns coordinate lists from a label
file that was generated using 
pull_images_and_labels.py
"""
def coordinates_from_label_file(label_file_name):    
    label_file = open(label_file_name, 'r')
    
    #Reads in labels into lists. 
    lat_points = []
    long_points = []

    for line in label_file:
        values = line.split(';')
        latitude = float(values[0])
        longitude = float(values[1])

        long_points.append(longitude)
        lat_points.append(latitude)

    label_file.close()

    return [lat_points, long_points]

"""
Reads and returns coordinate list from a .csv
file. 
"""
def coordinates_from_csv_file(csv_file_name):
    csv_file = open(csv_file_name, 'r')

    #Reads values into lists. 
    lat_points = []
    long_points = []

    for line in csv_file:
        if not line.startswith('Date'):   
            values = line.split(',')

            latitude = float(values[3])
            longitude = float(values[4])

            long_points.append(longitude)
            lat_points.append(latitude)

    csv_file.close()

    return [lat_points, long_points]

"""
Reads in and returns map specifications
from a map specifications file. 
"""
def read_in_overlay_specs(overlay_specs_file):
    specs_file = open(overlay_specs_file, 'r')

    #Reads in specs and returns them.
    specs = []

    for line in specs_file:
        specs.append(line.strip())

    specs_file.close()

    #Formats specs and returns them. 
    map_img_name = specs[0]
    BR = tuple([float(value) for value in specs[1].split(';')])
    TL = tuple([float(value) for value in specs[2].split(';')])

    return [map_img_name, BR, TL]

"""
Overlays a map onto gps coordinate points using
a map specification file and a file containing
gps coordinate points. 
"""
def overlay(overlay_specs_file, coordinate_file_name):
    #Determines coordinates of corner of map image. 
    map_img_name, BR, TL = read_in_overlay_specs(overlay_specs_file)

    #Loads map image onto plot using corner coordinates. 
    map_img_file = imread(map_img_name)

    plt.imshow(map_img_file, zorder=0, extent=[TL[1], BR[1], BR[0], TL[0]])

    #Gets coordinate points from run.
    #Assumes that anything not ending in .csv is a label file. 
    if coordinate_file_name.endswith('.csv'):
        lat_points, long_points = coordinates_from_csv_file(coordinate_file_name)
    else: 
        lat_points, long_points = coordinates_from_label_file(coordinate_file_name)

    #Plots coordinate points and shows plot. 
    plt.plot(long_points, lat_points, 'ro')

    plt.show()

if __name__ == "__main__":
    overlay(sys.argv[1], sys.argv[2])

#TODO Extend code below when needed for gps to xy coordinate translation. 
"""    
    x_points = []
    y_points = []

for i in range(len(lat_points)):
    h = VincentyDistance((BL[0], BL[1]), (lat_points[i], long_points[i])).meters
    e = VincentyDistance((BR[0], BR[1]), (lat_points[i], long_points[i])).meters
    d = VincentyDistance((BL[0], BL[1]), (BR[0], BR[1])).meters

    x = ((h ** 2) + (d ** 2) - (e ** 2)) / (2 * d)
    y = (h ** 2) - (x ** 2)

    x_points.append(x)
    y_points.append(y)
"""
