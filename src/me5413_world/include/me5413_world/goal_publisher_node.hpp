/* goal_publisher_node.hpp

 * Copyright (C) 2023 SS47816

 * Declarations for GoalPublisherNode class
 
**/

#ifndef GOAL_PUBLISHER_NODE_H_
#define GOAL_PUBLISHER_NODE_H_

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

namespace me5413_world 
{

class GoalPublisherNode
{
 public:
  GoalPublisherNode();
  virtual ~GoalPublisherNode() {};

 private:
  void timerCallback(const ros::TimerEvent&);
  void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void goalNameCallback(const std_msgs::String::ConstPtr& name);
  void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose);
  void boxMarkersCallback(const visualization_msgs::MarkerArray::ConstPtr& box_markers);
  
  tf2::Transform convertPoseToTransform(const geometry_msgs::Pose& pose);
  geometry_msgs::PoseStamped getGoalPoseFromConfig(const std::string& name);
  std::pair<double, double> calculatePoseError(const geometry_msgs::Pose& pose_robot, const geometry_msgs::Pose& pose_goal);

  // ROS declaration
  ros::NodeHandle nh_;
  ros::Timer timer_;

  ros::Publisher pub_goal_;
  ros::Publisher pub_absolute_position_error_;
  ros::Publisher pub_absolute_heading_error_;
  ros::Publisher pub_relative_position_error_;
  ros::Publisher pub_relative_heading_error_;

  ros::Subscriber sub_robot_odom_;
  ros::Subscriber sub_goal_name_;
  ros::Subscriber sub_goal_pose_;
  ros::Subscriber sub_box_markers_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_bcaster_;

  // Robot pose
  std::string world_frame_;
  std::string map_frame_;
  std::string robot_frame_;
  std::string goal_type_;

  geometry_msgs::Pose pose_world_robot_;
  geometry_msgs::Pose pose_world_goal_;
  geometry_msgs::Pose pose_map_robot_;
  geometry_msgs::Pose pose_map_goal_;
  std::vector<geometry_msgs::PoseStamped> box_poses_;

  std_msgs::Float32 absolute_position_error_;
  std_msgs::Float32 absolute_heading_error_;
  std_msgs::Float32 relative_position_error_;
  std_msgs::Float32 relative_heading_error_;
};

} // namespace me5413_world

#endif // GOAL_PUBLISHER_NODE_H_