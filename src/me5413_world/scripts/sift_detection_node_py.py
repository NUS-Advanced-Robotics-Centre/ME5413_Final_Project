#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf

class SIFTDetectionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.template_path = rospy.get_param("~template_path", "")
        if self.template_path:
            self.template_image = cv2.imread(self.template_path, cv2.IMREAD_GRAYSCALE)
        else:
            rospy.logerr("Template path is empty!")

        self.camera_matrix = None
        self.dist_coeffs = None
        self.display_image = None
        self.tf_listener = tf.TransformListener()

        self.image_sub = rospy.Subscriber("/front/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/front/camera_info", CameraInfo, self.camera_info_callback)
        self.goal_pub = rospy.Publisher("/detected_goal_pose", PoseStamped, queue_size=1)

        self.sift = cv2.xfeatures2d.SIFT_create()
        self.bf_matcher = cv2.BFMatcher()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        except Exception as e:
            rospy.logerr("cv_bridge exception: %s", e)
            return

        bbox = self.sift_detection(cv_image)
        self.display_image = cv_image.copy()
        self.visualize_detection(bbox)
        self.calculate_target(bbox)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)

    def sift_detection(self, image):
        # Detect keypoints and compute descriptors
        kp1, des1 = self.sift.detectAndCompute(self.template_image, None)
        kp2, des2 = self.sift.detectAndCompute(image, None)

        # Flann-based matcher
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        # Apply ratio test
        good_matches = []
        for m, n in matches:
            if m.distance < 0.50 * n.distance:
                good_matches.append(m)

        # Compute bounding box
        if len(good_matches) > 10:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

            if M is not None:
                h, w = self.template_image.shape
                pts = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)
                dst = cv2.perspectiveTransform(pts, M)
                dst = dst.reshape(-1, 2)  # Convert to 2D array
                x, y, w, h = cv2.boundingRect(dst)
                return (x, y, w, h)

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

        # Set a constant depth value
        depth = 0.1

        x, y, w, h = bbox
        target_point = np.array([(x + w / 2 - self.camera_matrix[0, 2]) * depth / self.camera_matrix[0, 0],
                                 (y + h / 2 - self.camera_matrix[1, 2]) * depth / self.camera_matrix[1, 1],
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
    rospy.init_node("sift_detection_node_py")
    sift_detection_node = SIFTDetectionNode()
    rospy.spin()

if __name__ == "__main__":
    main()