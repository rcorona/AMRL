#!/usr/bin/python

from geopy.distance import VincentyDistance
import sys
import rospy
from particle_filter.msg import Pose
from sensor_msgs.msg import NavSatFix
import random

#Globals
gps_origin = None
gps_pose = Pose()
gps_reading_buff = None
pub = None

#Globals for testing filter. 
global rand_threshold 
rand_threshold = 1.0

global num_threshold 
num_threshold = 500

global msg_counter 
msg_counter = 0

global distant_pose 
distant_pose = Pose()
distant_pose.x = 30.0
distant_pose.y = -30.0

def converter_callback(gps_reading):
    global gps_reading_buff
    global gps_origin
    global msg_counter

    #Update message count. 
    msg_counter += 1

    gps_reading_buff = gps_reading

    #Sets origin if first reading received. 
    if gps_origin == None:
        gps_origin = gps_reading

    
    pub.publish(pose_estimate())

def pose_estimate():
    global gps_pose
    global gps_reading_buff
    global gps_origin
    global msg_counter
    global num_threshold
    global distant_pose

    #Returns (0, 0) if no readings yet. 
    if gps_origin == None:
        gps_pose.x = 0.0
        gps_pose.y = 0.0
    else:
        origin = (gps_origin.latitude, gps_origin.longitude)

        #Otherwise calculates distance in meters to origin. 
        gps_pose.x = VincentyDistance(origin, (origin[0], gps_reading_buff.longitude)).meters
        gps_pose.y = VincentyDistance(origin, (gps_reading_buff.latitude, origin[1])).meters

        #Corrects signs if necessary. 
        if origin[1] > gps_reading_buff.longitude:
            gps_pose.x = -gps_pose.x

        if origin[0] > gps_reading_buff.latitude:
            gps_pose.y = -gps_pose.y

    if msg_counter > num_threshold:
        return distant_pose
    elif random.random() < rand_threshold:
        return gps_pose
    else:
        random_pose = Pose()
        random_pose.x = random.uniform(0, 280)
        random_pose.y = random.uniform(-100.0, 100.0)

        return random_pose

if __name__ == '__main__':
    #Init ROS and publisher. 
    rospy.init_node("gps_converter", anonymous=True)
    rate = rospy.Rate(10)

    global pub
    pub = rospy.Publisher("gps_pos_meters", Pose, queue_size=10)

    #Subscribe to raw GPS topic. 
    sub = rospy.Subscriber("navsat/fix", NavSatFix, converter_callback) 

    #Seeds random number generator (used for testing the addition of noise to reading).
    random.seed()

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()



