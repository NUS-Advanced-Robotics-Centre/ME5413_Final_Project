import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import enum
from skimage import filters
import argparse
from geometry_msgs.msg import Twist 

class Mode(enum.Enum):
    INFERENCE=0
    DATA_COLLECTION=1

class DigitDetector:
    CORRECT_DIGIT_THRESHOLD = 0.3
    CLOSE_TO_BOX_THRESHOLD = 0.6
    VEL_X = 0.2
    ANG_Z_KP = 0.15
    def __init__(self, digit):
        rospy.init_node('listener')
        self.digit = digit
        self.img_topic = '/front/image_raw'
        self.cmdvel_topic = 'cmd_vel'
        self.bridge = CvBridge()
        
        #self.templates = [f"template{i}.jpg" for i in range(5)]
        self.template_filename_fn = lambda id : f"digit_{self.digit}_template{id}.jpg"
        
        template_ids = [5,6,7,8]
        self.templates = [ self.template_filename_fn(id) for id in template_ids]
        
        
        self.img_sub = rospy.Subscriber(self.img_topic, Image, self.img_cb)
        self.cmdvel_pub = rospy.Publisher(self.cmdvel_topic, Twist, queue_size=1)
        self.raw_img = None
        
    def img_cb(self, raw_img):
        self.raw_img = raw_img
        
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
            
            if mode == Mode.DATA_COLLECTION:
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
            for template_dir in self.templates:
                template = cv2.imread(template_dir)
                
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
                
                if max_match_val >= self.CLOSE_TO_BOX_THRESHOLD:
                    rospy.signal_shutdown("Close enough to the box!")
                    break
                
                roi_center = (top_left[0] + w//2, top_left[1] + h//2)
                obs_center = (w_img // 2 , h_img // 2)
                
                vel_x = self.VEL_X
                yaw_rate = self.ANG_Z_KP * (-roi_center[0] + obs_center[0]) / (w_img//2)
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