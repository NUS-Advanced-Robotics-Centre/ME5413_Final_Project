#!/usr/bin/env python
import rospy
import os
import sys
from std_msgs.msg import Float32

class TrackErrorLogger:
    def __init__(self, csv_file_path):
        self.csv_file_path = csv_file_path
        self.position_error = None
        self.heading_error = None
        self.speed_error = None

        rospy.init_node('track_error_logger')
        rospy.Subscriber('/me5413_world/planning/rms_position_error', Float32, self.position_callback)
        rospy.Subscriber('/me5413_world/planning/rms_heading_error', Float32, self.heading_callback)
        rospy.Subscriber('/me5413_world/planning/rms_speed_error', Float32, self.speed_callback)

    def position_callback(self, msg):
        self.position_error = msg.data

    def heading_callback(self, msg):
        self.heading_error = msg.data

    def speed_callback(self, msg):
        self.speed_error = msg.data

    def run(self):
        rate = rospy.Rate(10)  # 10hz
        with open(self.csv_file_path, 'w') as f:
            while not rospy.is_shutdown():
                data = "{},{},{}\n".format(self.position_error, self.heading_error, self.speed_error)
                f.write(data)
                rate.sleep()

def main(csv_file_path):
    try:
        logger = TrackErrorLogger(csv_file_path)
        logger.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Please provide the CSV file path as a command-line argument.")
        sys.exit(1)

    csv_file_path = sys.argv[1]
    main(csv_file_path)