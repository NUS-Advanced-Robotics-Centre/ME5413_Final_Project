#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf

class Find2DObjectGoalPublisherNode:
    def __init__(self):
        self.objects_sub = rospy.Subscriber("/objects", Float32MultiArray, self.objects_callback)
        self.camera_info_sub = rospy.Subscriber("/front/camera_info", CameraInfo, self.camera_info_callback)
        
        self.goal_pub = rospy.Publisher("/detected_goal_pose", PoseStamped, queue_size=1)

        self.object_data = None
        self.camera_matrix = None

    def objects_callback(self, msg):
        if len(msg.data) == 0:
            return
        self.object_data = msg.data

        if self.object_data is not None:
            self.calculate_target()

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)

    def calculate_target(self):
        if self.camera_matrix is None:
            return

        data = self.object_data
        
        # Set a constant depth value
        depth = 0.2

        object_id = int(data[0])
        object_width = data[1]
        object_height = data[2]
        h31 = data[9]
        h32 = data[10]

        center_x = h31 + object_width/2
        center_y = h32 + object_height/2

        target_point = np.array([(center_x - self.camera_matrix[0, 2]) * depth / self.camera_matrix[0, 0],
                                 (center_y - self.camera_matrix[1, 2]) * depth / self.camera_matrix[1, 1],
                                 depth])

        target_point_camera = PointStamped()
        target_point_camera.header.stamp = rospy.Time.now()
        target_point_camera.header.frame_id = "front_camera_optical"
        target_point_camera.point.x = target_point[0]
        target_point_camera.point.y = target_point[1]
        target_point_camera.point.z = target_point[2]

        try:
            self.tf_listener.waitForTransform("map", target_point_camera.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
            target_point_map = self.tf_listener.transformPoint("map", target_point_camera)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: %s", e)
            return

        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position = target_point_map.point
        goal_pose.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_pose)

def main():
    rospy.init_node("find_2d_object_goal_publisher_node_py")
    template_matching_node = Find2DObjectGoalPublisherNode()
    rospy.spin()

if __name__ == "__main__":
    main()