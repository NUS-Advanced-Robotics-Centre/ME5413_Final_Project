#ifndef BOX_EXPLORER_NODE_H_
#define BOX_EXPLORER_NODE_H_

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

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32MultiArray.h>

namespace me5413_world
{

class BoxExplorerNode
{
 public:
  BoxExplorerNode();
  virtual ~BoxExplorerNode() {};

 private:
  void timerCallback(const ros::TimerEvent&);
  void goalNameCallback(const std_msgs::String::ConstPtr& name);
  void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose);
  void boxMarkersCallback(const visualization_msgs::MarkerArray::ConstPtr& box_markers);
  void globalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap);
  void goalPoseDetectedCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose);
  void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);

  std::vector<geometry_msgs::PoseStamped> createWaypoints();
  void updateCurrentWaypoint();
  void updateGoalIfReached();
  // bool isPointInObstacle(const geometry_msgs::PoseStamped& Point, const nav_msgs::OccupancyGrid& costmap);
  bool isPointInObstacle(const geometry_msgs::PoseStamped& Point);  
  tf2::Transform convertPoseToTransform(const geometry_msgs::Pose& pose);

  // ROS declaration
  ros::NodeHandle nh_;

  ros::Publisher pub_goal_;

  ros::Subscriber sub_goal_name_;
  ros::Subscriber sub_goal_pose_;
  ros::Subscriber sub_box_markers_;
  ros::Subscriber sub_global_costmap_;
  ros::Subscriber sub_objects_;
  ros::Subscriber sub_goal_pose_detected_;
  ros::Subscriber sub_robot_odom_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_bcaster_;

  // Robot pose
  std::string world_frame_;
  std::string map_frame_;
  std::string robot_frame_;
  std::string goal_type_;
  int goal_box_id_;

  geometry_msgs::Pose pose_world_robot_;
  geometry_msgs::Pose pose_world_goal_;
  geometry_msgs::Pose pose_map_robot_;
  geometry_msgs::Pose pose_map_goal_;
  std::vector<geometry_msgs::PoseStamped> box_poses_;

  // Global costmap
  nav_msgs::OccupancyGrid global_costmap_;
  
  // Waypoints
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  int current_waypoint_index_;

  // Goal detection
  bool is_goal_detected_;
};

} // namespace me5413_world

#endif // GOAL_PUBLISHER_NODE_H_