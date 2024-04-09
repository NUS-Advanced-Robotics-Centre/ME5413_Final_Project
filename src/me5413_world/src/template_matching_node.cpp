#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/calib3d.hpp>
#include <tf/transform_listener.h>

namespace me5413_world
{

class TemplateMatchingNode
{
public:
  TemplateMatchingNode();
  ~TemplateMatchingNode(){}

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_image_;
  image_transport::Subscriber sub_depth_image_;
  ros::Subscriber sub_camera_info_;
  ros::Publisher pub_goal_;
  std::string template_path_;
  cv::Mat template_image_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Mat cv_depth_image_;
  tf::TransformListener tf_listener_;
  cv::Mat display_image_;

  void imageCallback(const sensor_msgs::Image::ConstPtr& image);
  void depthCallback(const sensor_msgs::Image::ConstPtr& depth_image);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info);
  cv::Rect templateMatching(const cv::Mat& image);
  void visualizeDetection(const cv::Rect& bbox);
  void calculateTarget(const cv::Rect& bbox);
};

TemplateMatchingNode::TemplateMatchingNode() : it_(nh_)
{
  sub_image_ = it_.subscribe("/front/image_raw", 1, &TemplateMatchingNode::imageCallback, this);
  sub_depth_image_ = it_.subscribe("/front/depth/image_raw", 1, &TemplateMatchingNode::depthCallback, this);
  sub_camera_info_ = nh_.subscribe("/front/camera_info", 1, &TemplateMatchingNode::cameraInfoCallback, this);
  pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/detected_goal_pose", 1);

  ros::param::param<std::string>("~template_path", template_path_, "");
  if (!template_path_.empty()) {
    template_image_ = cv::imread(template_path_, cv::IMREAD_GRAYSCALE);
  } else {
    ROS_ERROR("Template path is empty!");
  }
}

void TemplateMatchingNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Convert the ROS image message to OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Rect bbox = templateMatching(cv_ptr->image);
  cv_ptr->image.copyTo(display_image_);
  visualizeDetection(bbox);
  calculateTarget(bbox);
}

void TemplateMatchingNode::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Convert the ROS image message to OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_depth_image_ = cv_ptr->image;
}

void TemplateMatchingNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->K.data());
  dist_coeffs_ = cv::Mat(1, 5, CV_64F, (void*)msg->D.data());
}

/**
 * @brief Template matching function
 * @param image The input image
 * @return The bounding box of the best match
 */
cv::Rect TemplateMatchingNode::templateMatching(const cv::Mat& image)
{
  std::vector<cv::Mat> pyramid;
  cv::Mat resized_image = image;
  pyramid.push_back(resized_image);
  for (int i = 0; i < 3; i++) {
    cv::pyrDown(resized_image, resized_image);
    pyramid.push_back(resized_image);
  }

  double best_score = 0;
  cv::Rect best_bbox;
  for (size_t i = 0; i < pyramid.size(); i++) {
    cv::Mat result;
    cv::matchTemplate(pyramid[i], template_image_, result, cv::TM_CCOEFF_NORMED);
    double min_val, max_val;
    cv::Point min_loc, max_loc;
    cv::minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc);
    if (max_val > best_score) {
      best_score = max_val;
      best_bbox = cv::Rect(max_loc.x, max_loc.y, template_image_.cols, template_image_.rows);
      best_bbox.x *= std::pow(2, pyramid.size() - 1 - i);
      best_bbox.y *= std::pow(2, pyramid.size() - 1 - i);
      best_bbox.width *= std::pow(2, pyramid.size() - 1 - i);
      best_bbox.height *= std::pow(2, pyramid.size() - 1 - i);
    }
  }

  return best_bbox;
}

/**
 * @brief Visualization function
 * @param image The input image (will be modified with the visualization)
 * @param bbox The bounding box of the detection
 */
void TemplateMatchingNode::visualizeDetection(const cv::Rect& bbox)
{
  cv::rectangle(display_image_, bbox, cv::Scalar(0, 255, 0), 2);
  cv::putText(display_image_, "Target", cv::Point(bbox.x, bbox.y - 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
  cv::imshow("Detection Result", display_image_);
  cv::waitKey(1);
}

/**
 * @brief Camera calibration function
 * @param bbox The bounding box of the detection
 * @return The 3D coordinates of the target point
 */
/**
 * @brief Calculate the 3D position of the target and publish it as a goal
 * @param bbox The bounding box of the detection
 */
void TemplateMatchingNode::calculateTarget(const cv::Rect& bbox)
{
  if (cv_depth_image_.empty()) {
    ROS_WARN("Depth image is empty.");
    return;
  }

  // Get the depth value at the center of the bounding box
  float depth = cv_depth_image_.at<float>(bbox.y + bbox.height / 2, bbox.x + bbox.width / 2);

  if (std::isnan(depth) || std::isinf(depth)) {
    ROS_WARN("Invalid depth value.");
    return;
  }

  // Calculate the 3D position of the target in the camera frame
  cv::Point3f target_point;
  target_point.x = (bbox.x + bbox.width / 2 - camera_matrix_.at<double>(0, 2)) * depth / camera_matrix_.at<double>(0, 0);
  target_point.y = (bbox.y + bbox.height / 2 - camera_matrix_.at<double>(1, 2)) * depth / camera_matrix_.at<double>(1, 1);
  target_point.z = depth;

  // Transform the target point to the map frame
  geometry_msgs::PointStamped target_point_camera;
  target_point_camera.header.stamp = ros::Time::now();
  target_point_camera.header.frame_id = "camera_color_optical_frame";
  target_point_camera.point.x = target_point.x;
  target_point_camera.point.y = target_point.y;
  target_point_camera.point.z = target_point.z;

  geometry_msgs::PointStamped target_point_map;
  try {
    tf_listener_.waitForTransform("map", target_point_camera.header.frame_id, ros::Time::now(), ros::Duration(1.0));
    tf_listener_.transformPoint("map", target_point_camera, target_point_map);
  }
  catch (tf::TransformException& ex) {
    ROS_ERROR("Transform error: %s", ex.what());
    return;
  }

  // Publish the goal pose
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.stamp = ros::Time::now();
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position = target_point_map.point;
  goal_pose.pose.orientation.w = 1.0;
  pub_goal_.publish(goal_pose);
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "template_matching_node");
  me5413_world::TemplateMatchingNode template_matching_node;
  ros::spin();
  return 0;
}