/* goal_publisher_node.cpp

 * Copyright (C) 2023 SS47816

 * ROS Node for publishing goal poses 
 
**/

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace interactive_tools 
{

class GoalPublisherNode
{
 public:
  GoalPublisherNode();
  virtual ~GoalPublisherNode() {};

 private:
  // Helper function to construct pose msgs
  void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void goalPoseCallback(const std_msgs::String& name);
  geometry_msgs::PoseStamped getPoseMsgFromConfig(const std::string& name);

  // ROS declaration
  ros::NodeHandle nh_;
  ros::Publisher pub_goal_;
  ros::Subscriber sub_robot_odom_;
  ros::Subscriber sub_goal_name_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  // Robot pose
  std::string world_frame_;
  std::string map_frame_;
  std::string robot_frame_;
  geometry_msgs::Pose robot_pose_;
};

GoalPublisherNode::GoalPublisherNode() : tf2_listener_(tf2_buffer_)
{
  pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &GoalPublisherNode::robotOdomCallback, this);
  sub_goal_name_ = nh_.subscribe("/goal_name", 1, &GoalPublisherNode::goalPoseCallback, this);

  // Initialization
  robot_frame_ = "base_link";
  map_frame_ = "map";
  world_frame_ = "world";
};

void GoalPublisherNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  // world_frame_ = odom->header.frame_id;
  // robot_frame_ = odom->child_frame_id;
  this->robot_pose_ = odom->pose.pose;

  
  return;
};

void GoalPublisherNode::goalPoseCallback(const std_msgs::String& name)
{
  const auto pose_msg = getPoseMsgFromConfig(name.data);
  this->pub_goal_.publish(pose_msg);
  return;
};

geometry_msgs::PoseStamped GoalPublisherNode::getPoseMsgFromConfig(const std::string& name)
{
  /** 
   * Get the Transform from goal to world from the file
   */
  double x, y, yaw;
  nh_.getParam("/me5413_project" + name + "/x", x);
  nh_.getParam("/me5413_project" + name + "/y", y);
  nh_.getParam("/me5413_project" + name + "/yaw", yaw);
  nh_.getParam("/me5413_project/frame_id", this->world_frame_);

  // tf2::Quaternion q;
  // q.setRPY(0, 0, yaw);
  // q.normalize();

  // geometry_msgs::Pose P_world_goal;
  // P_world_goal.position.x = x;
  // P_world_goal.position.y = y;
  // P_world_goal.orientation = tf2::toMsg(q);

  tf2::Transform T_world_goal;
  T_world_goal.setOrigin(tf2::Vector3(x, y, 0));
  tf2::Quaternion q_world_goal;
  q_world_goal.setRPY(0, 0, yaw);
  T_world_goal.setRotation(q_world_goal);

  /** 
   * Get the Transform from robot to world from the topic
   */
  std::cout << "fromMsg() 1" << std::endl;
  tf2::Transform T_world_robot;
  // tf2::fromMsg<geometry_msgs::Pose, tf2::Transform>(this->robot_pose_, T_world_robot);

  T_world_robot.setOrigin(tf2::Vector3(this->robot_pose_.position.x, this->robot_pose_.position.y, 0));
  tf2::Quaternion q_world_robot;
  q_world_robot.setValue(this->robot_pose_.orientation.x, this->robot_pose_.orientation.y, this->robot_pose_.orientation.z, this->robot_pose_.orientation.w);
  T_world_robot.setRotation(q_world_robot);

  /** 
   * Get the Pose of robot in map from the tf_listener
   */
  geometry_msgs::TransformStamped transform_map_robot;
  try
  {
      transform_map_robot = tf2_buffer_.lookupTransform(this->map_frame_, this->robot_frame_, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
      ROS_WARN("%s", ex.what());
      return geometry_msgs::PoseStamped();
  }

  std::cout << "fromMsg() 2" << std::endl;
  tf2::Transform T_map_robot;
  // tf2::fromMsg<geometry_msgs::TransformStamped, tf2::Transform>(transform_map_robot, T_map_robot);
  T_map_robot.setOrigin(tf2::Vector3(transform_map_robot.transform.translation.x, transform_map_robot.transform.translation.y, 0));
  tf2::Quaternion q_map_robot;
  q_map_robot.setValue(transform_map_robot.transform.rotation.x, transform_map_robot.transform.rotation.y, transform_map_robot.transform.rotation.z, transform_map_robot.transform.rotation.w);
  T_map_robot.setRotation(q_map_robot);
  std::cout << "fromMsg() 3" << std::endl;
  // geometry_msgs::Pose P_map_robot;
  // fromMsg(T_map_robot, P_map_robot);

  /** 
   * Calculate Transform from goal to map
   * T_map_goal = T_world_goal * T_world_robot^(-1) * T_map_robot
   */
  const tf2::Transform T_map_goal = T_world_goal * T_world_robot.inverse() * T_map_robot;

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = map_frame_;
  pose_msg.pose.position.x = T_map_goal.getOrigin().x();
  pose_msg.pose.position.y = T_map_goal.getOrigin().y();
  pose_msg.pose.orientation.x = T_map_goal.getRotation().getX();
  pose_msg.pose.orientation.y = T_map_goal.getRotation().getY();
  pose_msg.pose.orientation.z = T_map_goal.getRotation().getZ();
  pose_msg.pose.orientation.w = T_map_goal.getRotation().getW();

  return pose_msg;
};

} // namespace lgsvl_utils

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_publisher_node");
  interactive_tools::GoalPublisherNode goal_publisher_node;
  ros::spin();  // spin the ros node.
  return 0;
}