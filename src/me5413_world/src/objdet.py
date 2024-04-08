import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import enum
from skimage import filters
import argparse
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan
import os

class Mode(enum.Enum):
    INFERENCE=0
    DATA_COLLECTION=1

class DigitDetector:
    CORRECT_DIGIT_THRESHOLD = 0.3
    CLOSE_TO_BOX_THRESHOLD = 0.7
    STOP_DISTANCE_M = 1.0
    FRONT_MAX_ANGLE_RAD = 7.5 * np.pi / 180
    MAX_LIN_VEL = 1.0
    ANG_Z_KP = 0.5
    ETA = 1e-1
    MAX_LASER_DISTANCE_M = 10.0
    def __init__(self, digit):
        rospy.init_node('listener')
        self.digit = digit
        self.templates_folder = f'./digit_{self.digit}_templates/'
        self.img_topic = '/front/image_raw'
        self.laser_topic = '/front/scan'
        self.cmdvel_topic = 'cmd_vel'
        self.bridge = CvBridge()
        
        self.template_filename_fn = lambda id : f"digit_{self.digit}_template{id}.jpg"
        
        self.img_sub = rospy.Subscriber(self.img_topic, Image, self.img_cb)
        self.laser_sub = rospy.Subscriber(self.laser_topic, LaserScan, self.laser_cb)
        self.cmdvel_pub = rospy.Publisher(self.cmdvel_topic, Twist, queue_size=1)
        self.raw_img = None
        self.front_distance = None
        
    def img_cb(self, raw_img):
        self.raw_img = raw_img
        
    def laser_cb(self, laser_msg):
        self.front_distance = self.get_front_distance(laser_msg, self.FRONT_MAX_ANGLE_RAD)
        
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
        
    def run(self, mode : Mode = Mode.INFERENCE):
        template_id = 0
        while not rospy.is_shutdown():
            if self.raw_img is None:
                continue
            obs = self.bridge.imgmsg_to_cv2(self.raw_img, desired_encoding='passthrough')
            display = cv2.cvtColor(obs, cv2.COLOR_BGR2RGB)
            obs = cv2.cvtColor(obs, cv2.COLOR_BGR2GRAY)
            # obs = cv2.threshold(obs, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
            thresh = obs.copy()
            # obs = 255 - obs
            invert = obs.copy()
            
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
            
            thres_candidate = filters.threshold_otsu(obs)
            _,candidate_bin= cv2.threshold(obs,thres_candidate,255,cv2.THRESH_BINARY)
            
            max_match_val = None
            max_match_loc = None
            max_template = None
            for template_dir in os.listdir(self.templates_folder):
                template = cv2.imread(f'{self.templates_folder}{template_dir}')
                
                template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
                thres_template = filters.threshold_otsu(template_gray)
                _,template_bin = cv2.threshold(template_gray,thres_template,255,cv2.THRESH_BINARY)

                result = cv2.matchTemplate(candidate_bin, template_bin, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(result)
                if (max_match_val is None or max_val > max_match_val):
                    max_match_val = max_val
                    max_match_loc = max_loc
                    max_template = template_gray

            print("Maximum value detected ", max_match_val)
            if max_match_val >= self.CORRECT_DIGIT_THRESHOLD:
                print("Successful detection with score ", max_match_val)
                h,w = max_template.shape
                
                h_img, w_img = obs.shape
                top_left = max_match_loc
                bottom_right = (top_left[0] + w, top_left[1] + h)
                cv2.rectangle(display, top_left, bottom_right,(255,0,255), 2)
                
                roi_center = (top_left[0] + w//2, top_left[1] + h//2)
                obs_center = (w_img // 2 , h_img // 2)
                
                front_distance_error = self.front_distance - self.STOP_DISTANCE_M
                vel_x = min(front_distance_error, 1.0) * self.MAX_LIN_VEL
                
                x_center_error = (obs_center[0] - roi_center[0]) / (w_img//2)
                yaw_rate = x_center_error * self.ANG_Z_KP
                
                still_need_rotate = abs(x_center_error) > self.ETA
                if still_need_rotate:
                    vel_x = 0.0
                    
                still_move_forward = abs(vel_x) > self.ETA
                if not still_need_rotate and not still_move_forward:
                    status = f"Close enough to the box, front distance {self.front_distance} yaw_error {x_center_error}"
                    print(status)
                    rospy.signal_shutdown(status)
                    break
                
                print(f"Moving to box {vel_x, yaw_rate}...")
                self.cmdvel_pub.publish(self.get_twist_msg(vel_x, yaw_rate))
                    
                display_template = max_template
                color = (0,255,0)
            else:
                display_template = np.zeros(obs.shape)
                color = (0,0,255)
                print(f"No detections, rotating on the spot...")
                self.cmdvel_pub.publish(self.get_twist_msg(0.0, self.ANG_Z_KP))
            cv2.imshow("template", display_template)
            
            # Write the maximum template match score on the img display
            display = cv2.putText(display, str(round(max_match_val,2)), (50,50), cv2.FONT_HERSHEY_SIMPLEX,  
                   1, color, 2, cv2.LINE_AA) 
            
            cv2.imshow("obs", display)
            cv2.imshow("thresh", thresh)
            cv2.imshow("invert", invert)
            cv2.waitKey(1) 
        cv2.destroyAllWindows()
        

def main():
    detector = DigitDetector(3)
    parser = argparse.ArgumentParser("simple_example")
    parser.add_argument("--mode", help="1 for data collection", type=int, default=0)
    args = parser.parse_args()
    mode = Mode(args.mode)
    print("Running node in mode ", mode)
    detector.run(mode)

if __name__ == "__main__":
    main()