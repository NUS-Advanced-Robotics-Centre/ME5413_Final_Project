#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf
from tf.transformations import quaternion_from_euler

class Find2DObjectGoalPublisherNode:
    def __init__(self):
        self.depth = rospy.get_param("~depth", 0.05)
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

        center_x = data[6]
        center_y = data[7]

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
        
        # Get the current robot position in the map frame
        try:
            robot_pose = PoseStamped()
            robot_pose.header.stamp = rospy.Time(0)
            robot_pose.header.frame_id = "base_link"
            self.tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            robot_pose_map = self.tf_listener.transformPose("map", robot_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: %s", e)
            return

        # Calculate the direction from the robot to the target point
        dx = target_point_map.point.x - robot_pose_map.pose.position.x
        dy = target_point_map.point.y - robot_pose_map.pose.position.y
        yaw = np.arctan2(dy, dx)

        # Convert yaw to quaternion
        quat = quaternion_from_euler(0, 0, yaw)

        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position = target_point_map.point
        goal_pose.pose.orientation.w = quat[3]

        self.goal_pub.publish(goal_pose)

def main():
    rospy.init_node("find_2d_box_node_py")
    template_matching_node = Find2DObjectGoalPublisherNode()
    rospy.spin()

if __name__ == "__main__":
    main()