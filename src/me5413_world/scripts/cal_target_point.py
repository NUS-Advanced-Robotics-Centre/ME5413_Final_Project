#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
import numpy as np

class cal_target_point:
    def __init__(self):
        rospy.init_node('calculator_to_target')

        self.listener = tf.TransformListener()

        self.camera_info_sub = rospy.Subscriber('/front/rgb/camera_info', CameraInfo, self.camera_info_callback)
        self.depth_image_sub = rospy.Subscriber('/front/depth/image_raw', Image, self.depth_callback)
        self.rgb_image_sub = rospy.Subscriber('/front/rgb/image_raw', Image, self.rgb_callback)

        self.detection_sub = rospy.Subscriber('/objects', Float32MultiArray, self.detection_callback)

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.goal_name_pub = rospy.Publisher('/rviz_panel/goal_name', String, queue_size=10)
        self.calculated_target_pub = rospy.Publisher('/target_goal', PoseStamped, queue_size=10)

        self.bridge = CvBridge()
        self.camera_info = None
        self.cv_depth_image = None
        self.cv_rgb_image = None

    def camera_info_callback(self, data):
        self.camera_info = data

    def depth_callback(self, data):
        self.cv_depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")

    def rgb_callback(self, data):
        self.cv_rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def detection_callback(self, detection_msg):
        if self.camera_info is None or self.cv_depth_image is None:
            rospy.loginfo("Waiting for camera info and depth image...")
            return

        objects = np.array(detection_msg.data).reshape(-1, 12)

        for object_data in objects:
            object_id, bbox_width, bbox_height, *homography = object_data
            if bbox_width == 0 or bbox_height == 0:
                continue

            rospy.loginfo("Target found: Object ID {}".format(object_id))

            center_x = homography[6]  # dx in homography
            center_y = homography[7]  # dy in homography
            depth = self.cv_depth_image[int(center_y), int(center_x)]

            if np.isnan(depth) or np.isinf(depth):
                rospy.loginfo("Invalid depth value.")
                continue

            fx = self.camera_info.K[0]
            fy = self.camera_info.K[4]
            cx = self.camera_info.K[2]
            cy = self.camera_info.K[5]

            X = (center_x - cx) * depth / fx
            Y = (center_y - cy) * depth / fy
            Z = depth

            point_stamped = PointStamped()
            point_stamped.header.stamp = rospy.Time.now()  # Use the time when this point is being processed
            point_stamped.header.frame_id = "front_frame_optical"
            point_stamped.point.x = X
            point_stamped.point.y = Y
            point_stamped.point.z = Z

            try:
                # Adjust the waitForTransform to the time when the point is being processed
                self.listener.waitForTransform("map", "front_frame_optical", point_stamped.header.stamp,
                                               rospy.Duration(1.0))
                # Use the time when this point is being processed for the transform
                map_point = self.listener.transformPoint("map", point_stamped)

                goal_pose = PoseStamped()
                goal_pose.header.stamp = map_point.header.stamp
                goal_pose.header.frame_id = "map"
                goal_pose.pose.position = map_point.point
                goal_pose.pose.orientation.w = 1.0

                self.goal_pub.publish(goal_pose)
                self.calculated_target_pub.publish(goal_pose)

                goal_name_msg = String()
                goal_name_msg.data = "/done_1"
                self.goal_name_pub.publish(goal_name_msg)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr("TF error when converting point: %s", e)


if __name__ == '__main__':
    try:
        cal_target_point()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
