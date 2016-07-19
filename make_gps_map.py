#!/usr/bin/python

"""
Takes GPS data from a bag file and 
creates a vector.txt file for a map
to be used with the cobot localization
GUI. 
"""

import rospy
import rosbag
from geopy.distance import VincentyDistance
import sys

def get_vectors_every_n_meters(gps_msgs, n):
    #Sets origin to first gps reading.  
    origin = (gps_msgs[0].latitude, gps_msgs[0].longitude)

    reading_buffer = origin

    vectors = []

    for reading in gps_msgs:
        point = (reading.latitude, reading.longitude)

        #If >= to threshold, then write vector. 
        if VincentyDistance(reading_buffer, point).meters >= n:
            #Creates vector to be written to map. 
            start_x = VincentyDistance(origin, (origin[0], reading_buffer[1])).meters
            start_y = VincentyDistance(origin, (reading_buffer[0], origin[1])).meters

            end_x = VincentyDistance(origin, (origin[0], point[1])).meters
            end_y = VincentyDistance(origin, (point[0], origin[1])).meters

            #Assigns correct sign.
            if origin[0] > reading_buffer[0]:
                start_y = -start_y

            if origin[1] > reading_buffer[1]:
                start_x = -start_x

            if origin[0] > point[0]:
                end_y = -end_y

            if origin[1] > point[1]:
                end_x = -end_x

            vectors.append([start_x, start_y, end_x, end_y])

            #Updates buffer. 
            reading_buffer = point

    return vectors

def create_map(bag_file_name, map_file_name):
    bag = rosbag.Bag(bag_file_name, 'r')

    gps_msgs = []

    msg_counter = 1

    for topic, msg, t in bag.read_messages():
        if topic == '/navsat/fix':
            gps_msgs.append(msg)

            sys.stdout.write('\rRead ' + str(msg_counter) + ' messages.')
            sys.stdout.flush()
            msg_counter += 1

    #Puts cursor down one line. 
    print ''

    vectors = get_vectors_every_n_meters(gps_msgs, 0.5)

    #Writes vectors to map file. 
    map_file = open(map_file_name, 'w')

    for vector in vectors:
        vector_string = str(vector[0]) + ','
        vector_string += str(vector[1]) + ','
        vector_string += str(vector[2]) + ','
        vector_string += str(vector[3]) + '\n'

        map_file.write(vector_string)

    map_file.close()

if __name__ == '__main__':
    create_map(sys.argv[1], sys.argv[2])
