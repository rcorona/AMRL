#!/usr/bin/python

"""
This script takes a bag file and creates sets of images
labeled with their corresponding gps coordinates. 
The resulting file MUST still be pre-processed
in order to get class labels. The images must also 
be preprocessed to remove distortions and get them
to fit the required dimmensions. 
The images are pulled from the bag file every 'rate'
meters of movement. 

Usage: ./pull_images_and_gps.py [bag_file_name.bag] [rate] [left_camera_folder] [right_camera_folder]

Parameters: 
    bag_file_name: The name of the bag file to pull from. 
    rate: The rate in meters to pull (i.e. sample) images in. 
    left_camera_folder: The folder where the label file and corresponding jpg images will be kept
                        for the left camera. 
    right_camera_folder: Same as above, but for the right camera. 

Author: Rodolfo Corona, rcorona@utexas.edu
"""

import rospy
import rosbag
import sys
from math import sqrt

"""
Prepares the data for the image using the latest values 
in the bag file. 
"""
def write_data(camera_folder, data_file, latest_values, camera):
    #Writes image to jpg file in pertaining camera's folder. 
    img_file_name = camera_folder + str(latest_values['img_counters'][camera]) + '.jpg'
    img_file = open(img_file_name, 'w')

    img_file.write(latest_values['imgs'][camera].data)

    img_file.close()

    #Writes images label to the label file.
    data = str(latest_values['gps'].latitude) + ';'
    data += str(latest_values['gps'].longitude) + ';'
    data += img_file_name + '\n'

    data_file.write(label)

"""
The main method used to do the pulling. This is called from the main function,
but may be called seperately if being used in another script. 

The parameters correspond exactly to the ones specified at the top of this file (under Usage). 
"""
def pull_every_n_meters(file_name, rate, left_camera_folder, right_camera_folder):
    #Opens bag file.
    bag_file = rosbag.Bag(file_name, 'r')

    #Label files to build training set.
    left_data = open(left_camera_folder + 'raw_data.txt', 'w')
    right_data = open(right_camera_folder + 'raw_data.txt', 'w')

    #Topic names. 
    left_camera = '/camera_left/image_color/compressed'
    right_camera = '/camera_right/image_color/compressed'
    odom = '/jackal_velocity_controller/odom'
    gps = '/navsat/fix'

    #Converts to float for use in calculations. 
    rate = float(rate)

    #Keeps track of pertinent data. 
    odom_values = {'prev_x': 0.0, 'prev_y': 0.0, 'diff': 0.0}
    latest_values = {
                     'imgs': {'left': None, 'right': None}, 
                     'gps': None,
                     'img_counters': {'left': 0, 'right': 0}
                    }

    #Reads bag file to pull images. 
    for topic, msg, t in bag_file.read_messages():
        #First three topics merely update their respective reading. 
        if topic == left_camera:
            latest_values['imgs']['left'] = msg
        elif topic == right_camera:
            latest_values['imgs']['right'] = msg
        elif topic == gps:
            latest_values['gps'] = msg
        #If odometry, then check if movement rate has been met.
        elif topic == odom:
            #Calculates movement total since last reading. 
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)

            dx = odom_values['prev_x'] - x
            dy = odom_values['prev_y'] - y

            movement = sqrt((dx ** 2) + (dy ** 2))
            odom_values['diff'] += movement

            #Sets prev values to new values. 
            odom_values['prev_x'] = x
            odom_values['prev_y'] = y

            #If rate distance has been reached, pull image and reset counter. 
            if odom_values['diff'] >= rate:
                odom_values['diff'] = 0.0
        
                #Writes image label pairs for left and right cameras, updates their counters.  
                if not latest_values['imgs']['left'] == None:
                    write_data(left_camera_folder, left_data, latest_values, 'left')
                    latest_values['img_counters']['left'] += 1
                if not latest_values['imgs']['right'] == None:
                    write_data(right_camera_folder, right_data, latest_values, 'right')
                    latest_values['img_counters']['right'] += 1
                    

    #Closes files. 
    left_data.close()
    right_data.close()
    bag_file.close()

"""
If summoned from the command line, then just go through pulling
procedure. 
"""
if __name__ == "__main__":
    if not len(sys.argv) == 5:
        print 'Usage: ./pull_images_and_gps.py [bag_file_name.bag] [rate] [left_camera_folder] [right_camera_folder]'
    else:
        pull_every_n_meters(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
