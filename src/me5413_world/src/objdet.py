import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import enum
from skimage import filters
import argparse
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import os
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
import tf2_geometry_msgs
import tf2_ros

class Mode(enum.Enum):
    INFERENCE=0
    DATA_COLLECTION=1
    TESTING=2

class DetectionStrategy(enum.Enum):
    HISTOGRAM_TEMPLATE_MATCHING=0
    KNN_OCR=1
    
class Goal(int, enum.Enum):
    ASSEMBLY_LINE_1=0
    ROOM_BOX_ENTRANCE=1
    ROOM_BOX_3=2
    DELIVERY_VEHICLE_1=3
    
class DigitDetector:
    CORRECT_DIGIT_THRESHOLD = 0.5
    STOP_DISTANCE_M = 1.0
    FRONT_MAX_ANGLE_RAD = 7.5 * np.pi / 180
    MAX_LIN_VEL = 1.0
    MAX_YAW_RATE = 0.5
    MAX_LATERAL_ERROR = 0.2
    ETA = 1e-1
    SPOT_TURNING_YAW_RATE = 0.1
    MAX_LASER_DISTANCE_M = 10.0
    
    MAP_FRAME = 'map'
    WORLD_FRAME = 'world'
    ODOM_TOPIC = '/odometry/filtered'
    
    GOALS = [(4.0, -1.7, -1.57), (15.0, -2.0, -1.57), (0,0,0), (15.5, -8.2, -1.57)]
    
    def __init__(self, digit : int, strategy : DetectionStrategy):
        rospy.init_node('listener')
        self.digit = digit
        self.templates_folder = f'./digit_{self.digit}_templates/'
        self.img_topic = '/front/image_raw'
        self.laser_topic = '/front/scan'
        self.cmdvel_topic = 'cmd_vel'
        self.detection_topic = '/digitdetection'
        self.goal_topic = '/move_base_simple/goal'
        self.bridge = CvBridge()
        self.get_detections = self.get_detection_fn(strategy)
        
        self.template_filename_fn = lambda id : f"digit_{self.digit}_template{id}.jpg"
        
        self.img_sub = rospy.Subscriber(self.img_topic, Image, self.img_cb)
        self.laser_sub = rospy.Subscriber(self.laser_topic, LaserScan, self.laser_cb)
        self.cmdvel_pub = rospy.Publisher(self.cmdvel_topic, Twist, queue_size=1)
        self.detection_pub = rospy.Publisher(self.detection_topic, Image)

        # self.odom_sub = rospy.Subscriber(self.ODOM_TOPIC, Odometry, self.odom_cb)
        # self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped)
        # self.tfBuffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.cur_goal_id = 0
        
        self.raw_img = None
        self.front_distance = None
        self.robot_pose_world = None
        self.odom_msg = None
        
        self.first_goal_published = False
        
    def odom_cb(self, msg):
        self.odom_msg = msg
        
    def img_cb(self, msg):
        self.raw_img = msg
        
    def laser_cb(self, msg):
        self.front_distance = self.get_front_distance(msg, self.FRONT_MAX_ANGLE_RAD)
        
    @staticmethod
    def get_front_distance(laser_msg, max_angle_rad):
        max_angle_id = max_angle_rad / laser_msg.angle_increment
        middle_scan_id = len(laser_msg.ranges) // 2
        
        start_scan_id = max(0, middle_scan_id - int(max_angle_id))
        end_scan_id = min(len(laser_msg.ranges) - 1 , middle_scan_id + int(max_angle_id))
        
        for i in range(middle_scan_id, start_scan_id, -1):
            distance = laser_msg.ranges[i]
            if distance == np.inf:
                start_scan_id = i+1
                break
            
        for i in range(middle_scan_id, end_scan_id):
            distance = laser_msg.ranges[i]
            if distance == np.inf:
                end_scan_id = i-1
                break
        
        ranges_array = np.array(laser_msg.ranges)
        middle_laser_ranges = ranges_array[start_scan_id:end_scan_id]
        if len(middle_laser_ranges) == 0:
            return DigitDetector.MAX_LASER_DISTANCE_M
        return np.mean(middle_laser_ranges)
        
    @staticmethod
    def get_twist_msg(vel_x, yaw_rate):
        twist_msg = Twist()
        twist_msg.linear.x = vel_x
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = yaw_rate
        return twist_msg
    
    @staticmethod
    def get_roi_template_matching(obs, templates_folder, use_histogram=False):
        candidate = obs
        if use_histogram:
            thres_candidate = filters.threshold_otsu(obs)
            _, candidate= cv2.threshold(obs,thres_candidate,255,cv2.THRESH_BINARY)
        
        max_match_val = None
        max_match_loc = None
        max_template = None
        for template_dir in os.listdir(templates_folder):
            template = cv2.imread(f'{templates_folder}{template_dir}')
            
            template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            template_saved = template.copy()
            if use_histogram:
                thres_template = filters.threshold_otsu(template)
                _,template = cv2.threshold(template,thres_template,255,cv2.THRESH_BINARY)

            result = cv2.matchTemplate(candidate, template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(result)
            if (max_match_val is None or max_val > max_match_val):
                max_match_val = max_val
                max_match_loc = max_loc
                max_template = template_saved
        return max_match_val, max_match_loc, max_template
    
    @staticmethod
    def get_roi_knn_ocr(obs):
        return []*3
    
    def get_detection_fn(self, strategy : DetectionStrategy):
        if strategy == DetectionStrategy.HISTOGRAM_TEMPLATE_MATCHING:
            return lambda obs : DigitDetector.get_roi_template_matching(obs, self.templates_folder)
        elif strategy == DetectionStrategy.KNN_OCR:
            return lambda obs : DigitDetector.get_roi_knn_ocr(obs)
        
    def is_goal(self):
        (goal_x, goal_y, goal_yaw) = self.GOALS[self.cur_goal_id]
        if self.robot_pose_world is None:
            return False
        print("Goal is ", self.GOALS[self.cur_goal_id])
        print("Robot is ", self.robot_pose_world)
        (robot_x, robot_y, robot_yaw) = self.robot_pose_world
        if np.linalg.norm(np.array([robot_x, robot_y]) - np.array([goal_x, goal_y])) > self.ETA:
            return False
        elif abs(goal_yaw - robot_yaw) > self.ETA:
            return False
        return True
        
    def publish_goal(self):
        if self.odom_msg is None:
            return False
        goal_pose_world = PoseStamped()
        goal_pose_world.header.stamp = rospy.Time.now()
        goal_pose_world.header.frame_id = self.WORLD_FRAME
        x, y, yaw = self.GOALS[self.cur_goal_id]
        goal_pose_world.pose.position.x = x
        goal_pose_world.pose.position.y = y
        quat = quaternion_from_euler(0, 0, yaw)
        goal_pose_world.pose.orientation.x = quat[0]
        goal_pose_world.pose.orientation.y = quat[1]
        goal_pose_world.pose.orientation.z = quat[2]
        goal_pose_world.pose.orientation.w = quat[3]
        try:
            transform_map_world = self.tfBuffer.lookup_transform(self.MAP_FRAME, self.WORLD_FRAME, rospy.Time())
            transform_world_map = self.tfBuffer.lookup_transform(self.WORLD_FRAME, self.MAP_FRAME, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False
        # Transform the goal pose to map frame
        goal_pose_map = PoseStamped()
        goal_pose_map = tf2_geometry_msgs.do_transform_pose(goal_pose_world, transform_map_world)
        goal_pose_map.header.stamp = rospy.Time.now()
        goal_pose_map.header.frame_id = self.MAP_FRAME
        
        robot_pose_map = PoseStamped()
        robot_pose_map.header.stamp = rospy.Time.now()
        robot_pose_map.header.frame_id = self.MAP_FRAME
        robot_pose_map.pose.position.x = self.odom_msg.pose.pose.position.x
        robot_pose_map.pose.position.y = self.odom_msg.pose.pose.position.y
        
        orientation_q = self.odom_msg.pose.pose.orientation
        robot_pose_map.pose.orientation.x = orientation_q.x
        robot_pose_map.pose.orientation.y = orientation_q.y
        robot_pose_map.pose.orientation.z = orientation_q.z
        robot_pose_map.pose.orientation.w = orientation_q.w
        # Transform the robot pose to map frame
        robot_pose_world = tf2_geometry_msgs.do_transform_pose(robot_pose_map, transform_world_map)
        quat = robot_pose_world.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.robot_pose_world = (robot_pose_world.pose.position.x, robot_pose_world.pose.position.y, yaw)
        print("Published goal ", goal_pose_map)
        self.goal_pub.publish(goal_pose_map)
        return True
        
    def run(self, mode : Mode = Mode.INFERENCE):
        template_id = 0
        while not rospy.is_shutdown():
            # if self.cur_goal_id == len(self.GOALS):
            #     rospy.signal_shutdown("Finished navigation")
            #     break
            # if self.cur_goal_id != int(Goal.ROOM_BOX_3):
            #     if not self.first_goal_published and self.publish_goal():
            #         self.first_goal_published = True
            #     if self.is_goal():
            #         self.cur_goal_id +=1    
            #         self.publish_goal()
            #     continue
                
            if self.raw_img is None:
                continue
            display_lists = []

            obs = self.bridge.imgmsg_to_cv2(self.raw_img, desired_encoding='passthrough')
            display = cv2.cvtColor(obs, cv2.COLOR_BGR2RGB)
            obs = cv2.cvtColor(obs, cv2.COLOR_BGR2GRAY)
            
            ret1,th1 = cv2.threshold(obs,127,255,cv2.THRESH_BINARY)
            
            th1 = cv2.Canny(obs, threshold1 = 200, threshold2 = 200)
            
            # Otsu's thresholding
            ret2,th2 = cv2.threshold(obs,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            
            # Otsu's thresholding after Gaussian filtering
            blur = cv2.GaussianBlur(obs,(5,5),0)
            ret3,th3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            
            display_lists.append(("binary", th1))
            display_lists.append(("otsu", th2))
            display_lists.append(("otsu+gauss", th3))
            
            if mode == Mode.DATA_COLLECTION:
                print(f'Current front dist {self.front_distance}')
                cv2.imshow("obs", obs)
                key_pressed = cv2.waitKey(1) 
                if key_pressed & 0xFF == ord('q'):
                    # Press 'q' to quit
                    return                    
                elif key_pressed & 0xFF == ord('p'): 
                    # Press 'p' to take picture
                    filename = self.template_filename_fn(template_id)
                    cv2.imwrite(filename, obs)
                    print("Saved to ", filename)
                    template_id +=1
                continue
            
            max_match_val, max_match_loc, max_template = self.get_detections(obs)
            
            print("Maximum value detected ", max_match_val)
            vel_x, yaw_rate = 0.0, 0.0
            if max_match_val >= self.CORRECT_DIGIT_THRESHOLD:
                print("Successful detection with score ", max_match_val)
                h,w = max_template.shape
                
                h_img, w_img = obs.shape
                top_left = max_match_loc
                bottom_right = (top_left[0] + w, top_left[1] + h)
                cv2.rectangle(display, top_left, bottom_right,(255,0,255), 2)
                
                roi_center = (top_left[0] + w//2, top_left[1] + h//2)
                obs_center = (w_img // 2 , h_img // 2)
                
                lateral_error = (obs_center[0] - roi_center[0]) / (w_img//2)
                yaw_rate = lateral_error * self.MAX_YAW_RATE
                    
                front_distance_error = self.front_distance - self.STOP_DISTANCE_M
                vel_x = min(front_distance_error, 1.0) * self.MAX_LIN_VEL
                vel_x *= max(0, 1 - (abs(lateral_error)/self.MAX_LATERAL_ERROR))
                
                print("Front dist error, lateral error, ", front_distance_error, lateral_error, vel_x, yaw_rate)
                
                still_need_rotate = abs(lateral_error) > self.ETA
                still_move_forward = abs(vel_x) > self.ETA
                if not still_need_rotate and not still_move_forward:
                    print(f"Close enough to the box, front distance {self.front_distance} yaw_error {lateral_error}")
                    #self.cur_goal_id +=1     
                    rospy.signal_shutdown("Finished navigation")
                    break
                print(f"Moving to box...")
                display_template = max_template
                color = (0,255,0)
            else:
                display_template = np.zeros(obs.shape)
                color = (0,0,255)
                print(f"No detections, rotating on the spot...")
                yaw_rate = self.SPOT_TURNING_YAW_RATE
            
            if mode == Mode.TESTING:
                vel_x, yaw_rate = 0.0, 0.0                
            self.cmdvel_pub.publish(self.get_twist_msg(vel_x, yaw_rate))
            
            # Write the maximum template match score on the img display
            display = cv2.putText(display, str(round(max_match_val,2)), (50,50), cv2.FONT_HERSHEY_SIMPLEX,  
                   1, color, 2, cv2.LINE_AA) 
            
            self.detection_pub.publish(self.bridge.cv2_to_imgmsg(display, encoding="passthrough"))
            display_lists.append(("template", display_template))
            display_lists.append(("obs", display))
            for window_name, image in display_lists:
                cv2.imshow(window_name, image)
                cv2.waitKey(1) 
        cv2.destroyAllWindows()
        

def main():
    detector = DigitDetector(3, DetectionStrategy.HISTOGRAM_TEMPLATE_MATCHING)
    parser = argparse.ArgumentParser("simple_example")
    parser.add_argument("--mode", help="1 for data collection", type=int, default=0)
    args = parser.parse_args()
    mode = Mode(args.mode)
    print("Running node in mode ", mode)
    detector.run(mode)

if __name__ == "__main__":
    main()