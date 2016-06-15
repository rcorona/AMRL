#!/usr/bin/python

"""
This script clips a bag file using a start and end time (in seconds) within the file. 

Usage for single file: $ clip_bag_file.py single [in_file] [start time (s)] [end time (s)]
Usage for multiple files: $ clip_bag_file.py multiple [txt file with info]

Author: Rodolfo Corona, rcorona@utexas.edu
"""

import rospy
import rosbag
import sys
import yaml
import shutil

"""
Clips multiple bag files using a text file with their info as input.

txt file format:
    bag_file_1_name:clip_start_time;clip_end_time
    bag_file_2_name:clip_start_time;clip_end_time
    .
    .
    .
    bag_file_n_name:clip_start_time;clip_end_time

"""
def clip_bag_files(txt_file):
    info_file = open(txt_file, 'r')

    #Clips each file specified using values given. 
    for line in info_file:
        name_times = line.split(':')

        #Gets name, start, and end time for each file. 
        name = name_times[0]
        times = name_times[1].split(';')

        start_t = times[0]
        end_t = times[1].strip()

        print "Clipping " + name + "..."

        #Clips the bag file. 
        clip_bag_file(name, start_t, end_t)

"""
Clips a single file given its name, clip start
and end times. 
"""
def clip_bag_file(in_file, start_t, end_t):
    #Converts input to floats.
    start_t = float(start_t)
    end_t = float(end_t)

    #Opens bag files. 
    try:
        in_bag = rosbag.Bag(in_file, 'r')
    #If bag file doesn't exist, ignores it. 
    except IOError as e:
        print "Could not open file " + in_file + ", perhaps it was put in the wrong folder or spelled wrong?" 
        print "Ignoring and moving on to next file."

        return

    out_bag = rosbag.Bag(in_file.strip('.bag') + '.clipped', 'w')

    #Calculates start time. 
    bag_start_t = yaml.load(in_bag._get_yaml_info())['start'] 
    clip_start_t = bag_start_t + start_t

    #Calculates end time. 
    clip_duration = end_t - start_t
    clip_end_t = clip_start_t + clip_duration

    #Writes clip to out bag file. 
    for topic, msg, t in in_bag.read_messages():
        if t.to_sec() >= clip_start_t and t.to_sec() <= clip_end_t:
            out_bag.write(topic, msg, t)

    #Closes bag files. 
    in_bag.close()
    out_bag.close()

if __name__ == "__main__":
    def print_usage():
        
        print "Usage for single file: $ clip_bag_file.py single [in_file] [start time (s)] [end time (s)]"
        print "Usage for multiple files: $ clip_bag_file.py multiple [txt file with info]"

    if not len(sys.argv) >= 2:
        print_usage()
    elif sys.argv[1] == 'single':
        clip_bag_file(sys.argv[2], sys.argv[3], sys.argv[4])
    elif sys.argv[1] == 'multiple':
        clip_bag_files(sys.argv[2])
    else:
        print_usage()

