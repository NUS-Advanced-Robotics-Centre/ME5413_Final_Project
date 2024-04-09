#!/usr/bin/env python

import rospy
import rospkg
import os
from std_msgs.msg import Float32

position_error = None
heading_error = None
relative_position_error = None
relative_heading_error = None

def position_callback(msg):
    global position_error
    position_error = msg.data

def heading_callback(msg):
    global heading_error
    heading_error = msg.data

def relative_position_callback(msg):
    global relative_position_error
    relative_position_error = msg.data

def relative_heading_callback(msg):
    global relative_heading_error
    relative_heading_error = msg.data

def track_error_logger(bag_file_path):
    rospy.init_node('track_error_logger')
    rospy.Subscriber("/me5413_world/absolute/position_error", Float32, position_callback)
    rospy.Subscriber("/me5413_world/absolute/heading_error", Float32, heading_callback)
    rospy.Subscriber("/me5413_world/relative/position_error", Float32, relative_position_callback)
    rospy.Subscriber("/me5413_world/relative/heading_error", Float32, relative_heading_callback)

    rate = rospy.Rate(10)  # 10hz
    with open(bag_file_path, 'w') as f:
        while not rospy.is_shutdown():
            data = "{},{},{},{}\n".format(position_error, heading_error, relative_position_error, relative_heading_error)
            f.write(data)
            rate.sleep()

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('me5413_world')
    data_path = os.path.join(package_path, 'outputs/teb')
    # data_path = os.path.join(package_path, 'outputs/dwa')
    absolute_data_path = os.path.abspath(data_path)

    bag_file_path = os.path.join(absolute_data_path, 'teb_errors.csv')

    # bag_file_path = os.path.join(absolute_data_path, 'dwa_errors.csv')


    try:
        track_error_logger(bag_file_path)
    except rospy.ROSInterruptException:
        pass
