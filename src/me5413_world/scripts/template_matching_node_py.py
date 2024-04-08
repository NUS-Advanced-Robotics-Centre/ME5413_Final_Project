#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf
import os
from tf.transformations import quaternion_from_euler

class TemplateMatchingNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.template_folder = rospy.get_param("~template_folder", "")
        self.depth = rospy.get_param("~depth", 0.05)
        if self.template_folder:
            self.template_images = []
            for filename in os.listdir(self.template_folder):
                template_path = os.path.join(self.template_folder, filename)
                template_image = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
                if template_image is not None:
                    self.template_images.append(template_image)
        else:
            rospy.logerr("Template folder is empty!")

        self.camera_matrix = None
        self.dist_coeffs = None
        self.display_image = None
        self.tf_listener = tf.TransformListener()

        self.image_sub = rospy.Subscriber("/front/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/front/camera_info", CameraInfo, self.camera_info_callback)
        self.goal_pub = rospy.Publisher("/detected_goal_pose", PoseStamped, queue_size=1)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        except Exception as e:
            rospy.logerr("cv_bridge exception: %s", e)
            return

        bbox = self.template_matching(cv_image)
        self.display_image = cv_image.copy()
        self.visualize_detection(bbox)
        self.calculate_target(bbox)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)

    def template_matching(self, image):
        best_score = 0
        best_bbox = None

        for template_image in self.template_images:
            # Build the template pyramid
            template_pyramid = []
            resized_template = template_image
            scale_factor = [0.75, 1.0, 1.25, 1.5, 1.75]
            for s in scale_factor:
                resized_template = cv2.resize(resized_template, (0, 0), fx=s, fy=s)
                template_pyramid.append(resized_template)

            # Perform template matching
            for pyr_template in template_pyramid:
                result = cv2.matchTemplate(image, pyr_template, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(result)
                if max_val > best_score:
                    best_score = max_val
                    best_bbox = (max_loc[0], max_loc[1], pyr_template.shape[1], pyr_template.shape[0])

        # Confidence threshold
        confidence_threshold = 0.75
        if best_score >= confidence_threshold:
            return best_bbox
        else:
            return None

    def visualize_detection(self, bbox):
        if bbox is not None:
            x, y, w, h = bbox
            cv2.rectangle(self.display_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(self.display_image, "Target", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.imshow("Detection Result", self.display_image)
        cv2.waitKey(1)

    def calculate_target(self, bbox):
        if bbox is None:
            return
        if self.camera_matrix is None:
            rospy.logwarn("Camera matrix is not available!")
            return
        
        x, y, w, h = bbox
        target_point = np.array([(x + w / 2 - self.camera_matrix[0, 2]) * self.depth / self.camera_matrix[0, 0],
                                (y + h / 2 - self.camera_matrix[1, 2]) * self.depth / self.camera_matrix[1, 1],
                                self.depth])

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
    rospy.init_node("template_matching_node_py")
    template_matching_node = TemplateMatchingNode()
    rospy.spin()

if __name__ == "__main__":
    main()