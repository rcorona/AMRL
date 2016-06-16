#!/usr/bin/python

"""
This script overlays the gps points held within raw data
file (as prepared using pull_images_and_gps.py) or a csv file
for a jackal run onto a google map.

Usage: ./overlay_to_map.py [map_specs_file] [coordinate file] [bin size (m)]

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

    bin size - The size of the bins to be used for labelling the images 
               to feed the NN for training. This should be specified in meters. 

Author: Rodolfo Corona, rcorona@utexas.edu
"""

import rospy
import rosbag
import sys
from geopy.distance import VincentyDistance
import matplotlib.pyplot as plt
from scipy.misc import imread
import matplotlib.cbook as cbook
from math import sqrt
import math
import numpy as np

"""
Reads and returns coordinate lists from a raw
data file that was generated using 
pull_images_and_gps.py
"""
def coordinates_from_raw_data_file(label_file_name):    
    label_file = open(label_file_name, 'r')
    
    #Reads in labels into lists. 
    lat_points = []
    long_points = []

    for line in label_file:
        values = line.split(';')
        latitude = float(values[1])
        longitude = float(values[0])

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
Takes the top left and bottom right corners of the coordinate plane
in gps coordinates as well as lists of latitude and longitude
points to translate into a coordinate system. (0,0) will map to
the bottom left corner of the image. 
"""
def get_coordinates_in_meters(TL, BR, lat_points, long_points):
    x_points = []
    y_points = []

    height = float(VincentyDistance((BR[0], TL[1]), (TL[0], TL[1])).meters)
    width = float(VincentyDistance((BR[0], TL[1]), (BR[0], BR[1])).meters)

    #Translates each coordinate point based on formula below. 
    for i in range(len(lat_points)):
        h = float(VincentyDistance((BR[0], TL[1]), (lat_points[i], long_points[i])).meters)
        e = float(VincentyDistance((BR[0], BR[1]), (lat_points[i], long_points[i])).meters)

        x = ((h ** 2) + (width ** 2) - (e ** 2)) / (2 * width)
        y = sqrt((h ** 2) - (x ** 2))

        x_points.append(x)
        y_points.append(y)

    return [x_points, y_points, width, height]

"""
Creates a grid given a square bin length along with 
the height and width of the total map plot for visualization. 
Returns a list of labels pertaining to the xy pairs given. 
These labels may be used for training. 
"""
def grid_and_labels_from_bin_size(bin_length, figure, width, height, x_points, y_points):
    #Rounds up for even dimmensions. 
    width = int(math.ceil(width))
    height = int(math.ceil(height))
    
    #Sets grid on plot for visualization. 
    axes = figure.gca()
    axes.set_xticks(np.arange(0, width, bin_length))
    axes.set_yticks(np.arange(0, height, bin_length))
    plt.grid()

    #Gets dimmensions for bin matrix representation. 
    num_columns = abs(width / bin_length) + (1 if width % bin_length else 0)
    num_rows = (height / bin_length) + (1 if height % bin_length else 0)

    #Gets labels for each xy pair pertaining to bin numbers.  
    labels = []

    for i in range(len(x_points)):
        column = int(x_points[i]) / bin_length
        row = int(y_points[i]) / bin_length

        label = (row * num_columns) + column
        labels.append(label)

    return labels
    
"""
Overlays a map onto gps coordinate points using
a map specification file and a file containing
gps coordinate points. 
"""
def overlay(overlay_specs_file, coordinate_file_name, bin_size):
    #Determines coordinates of corner of map image. 
    map_img_name, BR, TL = read_in_overlay_specs(overlay_specs_file)

    #Gets coordinate points from run.
    #Assumes that anything not ending in .csv is a raw data file. 
    if coordinate_file_name.endswith('.csv'):
        lat_points, long_points = coordinates_from_csv_file(coordinate_file_name)
    else: 
        lat_points, long_points = coordinates_from_raw_data_file(coordinate_file_name)

    #Ensures same number of latitude and longitude points were gathered. 
    if not len(lat_points) == len(long_points):
        print "latitude and longitude points disjoint in size!!!"

        sys.exit()

    #Translates them into coordinate system in meters. 
    x_points, y_points, width, height = get_coordinates_in_meters(TL, BR, lat_points, long_points)

    #Loads map image onto plot using corner coordinates. 
    figure = plt.figure()
    map_img_file = imread(map_img_name)
    plt.imshow(map_img_file, zorder=0, extent=[0.0, width, 0.0, height])

    #Plots coordinate points and shows plot. 
    plt.plot(x_points, y_points, 'ro')

    #Creates grid that will correspond to location bins to train NN.
    #Also generates labels associated with the bins. 
    labels = grid_and_labels_from_bin_size(5, figure, width, height, x_points, y_points)

    #TODO write labels to file. 

    #Presents the plot. 
    plt.show()

if __name__ == "__main__":
    if not len(sys.argv) == 4:
        print 'Usage: ./overlay_to_map.py [map_specs_file] [coordinate file] [bin size (m)]' 
    else: 
        overlay(sys.argv[1], sys.argv[2], sys.argv[3])
