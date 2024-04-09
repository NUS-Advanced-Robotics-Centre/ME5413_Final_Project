#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

class BoxListener:
    def __init__(self):
        # Publisher for box3's position
        self.box_pose_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        # Subscriber for the box markers
        self.subscriber = rospy.Subscriber("/gazebo/ground_truth/box_markers", MarkerArray, self.boxMarkersCallback)
        # Timer for publishing box3's position
        self.publish_timer = None
        # Dictionary to store up to 9 box poses
        self.boxes = {}

    def boxMarkersCallback(self, data):
        # Store box poses until we have 9
        if len(self.boxes) < 9:
            for marker in data.markers:
                # Check if we already have this box
                if marker.id not in self.boxes:
                    self.boxes[marker.id] = marker.pose
                    rospy.loginfo("Box %d added.", marker.id)
                    # If it's box3 and we're not already publishing, start
                    if marker.id == 3 and (self.publish_timer is None or not self.publish_timer.is_alive()):
                        self.start_publishing(marker.pose)

    def start_publishing(self, box3_pose):

        rospy.loginfo("Starting to publish box3's position.")
        end_time = rospy.Time.now() + rospy.Duration(5)  # Set end time for 5 seconds from now
        rate = rospy.Rate(2)  # 2Hz, i.e., every 0.5 seconds
        while rospy.Time.now() < end_time:
            self.publish_box3_pose(box3_pose)
            rate.sleep()
        rospy.loginfo("Stopped publishing box3's position.")

    def publish_box3_pose(self, pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose = pose
        # pose_stamped.pose.position.y += 1
        self.box_pose_pub.publish(pose_stamped)

if __name__ == '__main__':
    rospy.init_node('listens', anonymous=True)
    box_listener = BoxListener()
    rospy.spin()
