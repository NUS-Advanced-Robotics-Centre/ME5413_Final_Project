/* goal_publisher_node.cpp

 * Copyright (C) 2023 SS47816

 * ROS Node for publishing goal poses

**/

#include "me5413_world/goal_publisher_node.hpp"

namespace me5413_world
{

GoalPublisherNode::GoalPublisherNode() : tf2_listener_(tf2_buffer_)
{
  this->pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  this->pub_absolute_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/absolute/position_error", 1);
  this->pub_absolute_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/absolute/heading_error", 1);
  this->pub_relative_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/relative/position_error", 1);
  this->pub_relative_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/relative/heading_error", 1);

  this->timer_ = nh_.createTimer(ros::Duration(0.2), &GoalPublisherNode::timerCallback, this);
  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &GoalPublisherNode::robotOdomCallback, this);
  this->sub_goal_name_ = nh_.subscribe("/rviz_panel/goal_name", 1, &GoalPublisherNode::goalNameCallback, this);
  this->sub_goal_pose_ = nh_.subscribe("/move_base_simple/goal", 1, &GoalPublisherNode::goalPoseCallback, this);
  this->sub_box_markers_ = nh_.subscribe("/gazebo/ground_truth/box_markers", 1, &GoalPublisherNode::boxMarkersCallback, this);
  this->sub_move_base_status_ = nh_.subscribe("/move_base/status", 10, &GoalPublisherNode::moveBaseStatusCallback, this);
  this->sub_map_ = nh_.subscribe("/move_base/global_costmap/costmap", 10, &GoalPublisherNode::globalCostmapCallback, this);
  this->sub_calculated_target_ = nh_.subscribe("/target_goal", 1, &GoalPublisherNode::calculatedTargetCallback, this);

  // Initialization
  this->robot_frame_ = "base_link";
  this->map_frame_ = "map";
  this->world_frame_ = "world";
  this->absolute_position_error_.data = 0.0;
  this->absolute_heading_error_.data = 0.0;
  this->relative_position_error_.data = 0.0;
  this->relative_heading_error_.data = 0.0;
  this->last_responded_goal_id_ = "";
  this->last_goal_time_ = ros::Time::now();
  this->globalCostmapData = nav_msgs::OccupancyGrid();
  this->gt_box_pose = geometry_msgs::PoseStamped();
  this->calculated_target_ = geometry_msgs::PoseStamped();
};

void GoalPublisherNode::timerCallback(const ros::TimerEvent&)
{
  // Calculate absolute errors (wrt to world frame)
  const std::pair<double, double> error_absolute = calculatePoseError(this->pose_world_robot_, this->pose_world_goal_);
  // Calculate relative errors (wrt to map frame)
  const std::pair<double, double> error_relative = calculatePoseError(this->pose_map_robot_, this->pose_map_goal_);

  this->absolute_position_error_.data = error_absolute.first;
  this->absolute_heading_error_.data = error_absolute.second;
  this->relative_position_error_.data = error_relative.first;
  this->relative_heading_error_.data = error_relative.second;

  if (this->goal_type_ == "box")
  {
    this->absolute_heading_error_.data = 0.0;
    this->relative_heading_error_.data = 0.0;
  }

  // Publish errors
  this->pub_absolute_position_error_.publish(this->absolute_position_error_);
  this->pub_absolute_heading_error_.publish(this->absolute_heading_error_);
  this->pub_relative_position_error_.publish(this->relative_position_error_);
  this->pub_relative_heading_error_.publish(this->relative_heading_error_);

  return;
};

void GoalPublisherNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->pose_world_robot_ = odom->pose.pose;

  const tf2::Transform T_world_robot = convertPoseToTransform(this->pose_world_robot_);
  const tf2::Transform T_robot_world = T_world_robot.inverse();

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = this->robot_frame_;
  transformStamped.child_frame_id = this->world_frame_;
  transformStamped.transform.translation.x = T_robot_world.getOrigin().getX();
  transformStamped.transform.translation.y = T_robot_world.getOrigin().getY();
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = T_robot_world.getRotation().getX();
  transformStamped.transform.rotation.y = T_robot_world.getRotation().getY();
  transformStamped.transform.rotation.z = T_robot_world.getRotation().getZ();
  transformStamped.transform.rotation.w = T_robot_world.getRotation().getW();

  this->tf2_bcaster_.sendTransform(transformStamped);

  return;
};

void GoalPublisherNode::calculatedTargetCallback(const geometry_msgs::PoseStamped& target)
{
  this->calculated_target_.pose = target.pose;
  return;
};

void GoalPublisherNode::goalNameCallback(const std_msgs::String::ConstPtr& name)
{
  const std::string goal_name = name->data;
  const int end = goal_name.find_last_of("_");
  this->goal_type_ = goal_name.substr(1, end - 1);
  const int goal_box_id = stoi(goal_name.substr(end + 1, 1));

  geometry_msgs::PoseStamped P_world_goal;
  if (this->goal_type_ == "box")
  {
    if (box_poses_.empty())
    {
      ROS_ERROR_STREAM("Box poses unknown, please spawn boxes first!");
      return;
    }
    else if (goal_box_id >= box_poses_.size())
    {
      ROS_ERROR_STREAM("Box id is outside the available range, please select a smaller id!");
      return;
    }

    P_world_goal = getRandomTargetInPackingArea();
    // The following line is commented out because the goal pose is now randomly generated
    // It represents the ground truth pose of the box in the world frame

    this->gt_box_pose = box_poses_[goal_box_id - 1];
  }
  else if (this->goal_type_ == "done")
  {
    P_world_goal = this->gt_box_pose;
    // The box search is done, so the robot should stay still
    // return;
  }
  else
  {
    // Get the Pose of the goal in world frame
    P_world_goal = getGoalPoseFromConfig(goal_name);
  }

  this->pose_world_goal_ = P_world_goal.pose;
  // Get the Transform from world to map from the tf_listener
  geometry_msgs::TransformStamped transform_map_world;
  try
  {
    transform_map_world = this->tf2_buffer_.lookupTransform(this->map_frame_, this->world_frame_, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  // Transform the goal pose to map frame
  geometry_msgs::PoseStamped P_map_goal;
  tf2::doTransform(P_world_goal, P_map_goal, transform_map_world);
  P_map_goal.header.stamp = ros::Time::now();
  P_map_goal.header.frame_id = map_frame_;

  // Transform the robot pose to map frame
  tf2::doTransform(this->pose_world_robot_, this->pose_map_robot_, transform_map_world);

  if (this->goal_type_ == "done")
  {
    double x1 = this->calculated_target_.pose.position.x;
    double y1 = this->calculated_target_.pose.position.y;
    double z1 = this->calculated_target_.pose.position.z;

    double x2 = P_world_goal.pose.position.x;
    double y2 = P_world_goal.pose.position.y;
    double z2 = P_world_goal.pose.position.z;
    ROS_INFO("Ground truth position is: x=%f, y=%f, z=%f", x2, y2, z2);

    // 计算差值
    double x_diff = x1 - x2;
    double y_diff = y1 - y2;
    double z_diff = z1 - z2;
    // Euclidean distance for position error
    double position_error = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
    ROS_INFO("Position error: %f", position_error);

    return;
  }

  // Publish goal pose in map frame
  this->pub_goal_.publish(P_map_goal);

  return;
};

// This callback is implemented to detect the status of the move_base action server
// It would give a new random goal if the previous goal is either succeeded or aborted
void GoalPublisherNode::moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray& status)
{
  // std::lock_guard<std::mutex> lock(mutex_);
  // Go through the status list
  for (const auto& goalStatus : status.status_list)
  {
    // Ensure the goal id is not the same as the last responded goal id
    double time_diff = (ros::Time::now() - last_goal_time_).toSec();
    // std::cout << "Time difference: " << time_diff << std::endl;
    if (goalStatus.goal_id.id != last_responded_goal_id_ &&
            time_diff > 0.3)
    {
      last_goal_time_ = ros::Time::now();
      if (goalStatus.status == actionlib_msgs::GoalStatus::SUCCEEDED ||
          goalStatus.status == actionlib_msgs::GoalStatus::ABORTED)
      {
        // std::cout << "Goal Type: " << this->goal_type_ << std::endl;
        if (this->goal_type_ == "box")
        {
          this->last_responded_goal_id_ = goalStatus.goal_id.id;

          // All conditions satisfied, target still not found, generate a new random goal
          geometry_msgs::PoseStamped newGoal = this->getRandomTargetInPackingArea();

          // Transform the new goal to map frame
          geometry_msgs::TransformStamped transform_map_world;
          try
          {
            transform_map_world = this->tf2_buffer_.lookupTransform(this->map_frame_, this->world_frame_, ros::Time(0));
          }
          catch (tf2::TransformException& ex)
          {
            ROS_WARN("%s", ex.what());
            return;
          }
          geometry_msgs::PoseStamped P_map_goal;
          tf2::doTransform(newGoal, P_map_goal, transform_map_world);
          P_map_goal.header.stamp = ros::Time::now();
          P_map_goal.header.frame_id = map_frame_;

          // Publish the new random goal
          this->pub_goal_.publish(P_map_goal);

          break;
        }
      }
    }
  }
}

void GoalPublisherNode::goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose)
{
  this->pose_map_goal_ = goal_pose->pose;
}

tf2::Transform GoalPublisherNode::convertPoseToTransform(const geometry_msgs::Pose& pose)
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, 0));
  tf2::Quaternion q;
  q.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  T.setRotation(q);

  return T;
};

void GoalPublisherNode::boxMarkersCallback(const visualization_msgs::MarkerArray::ConstPtr& box_markers)
{
  this->box_poses_.clear();
  for (const auto& box : box_markers->markers)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose = box.pose;
    this->box_poses_.emplace_back(pose);
  }

  return;
};

void GoalPublisherNode::globalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  this->globalCostmapData = *msg;  // Monitor the global costmap data
}

bool GoalPublisherNode::isPointInObstacle(double x, double y)
{
  // Calculate the map coordinates of the target point
  float resolution = this->globalCostmapData.info.resolution;
  int map_x = static_cast<int>((x - this->globalCostmapData.info.origin.position.x) / resolution);
  int map_y = static_cast<int>((y - this->globalCostmapData.info.origin.position.y) / resolution);

  // Define the radius of the circle to check
  double check_radius = 0.3;
  int radius_cells = static_cast<int>(check_radius / resolution);

  for (int dx = -radius_cells; dx <= radius_cells; dx++)
  {
    for (int dy = -radius_cells; dy <= radius_cells; dy++)
    {
      int check_x = map_x + dx;
      int check_y = map_y + dy;

      // Ensure the point is within the circle
      if (dx * dx + dy * dy <= radius_cells * radius_cells)
      {
        // Ignore points outside the map
        if (check_x < 0 || check_x >= this->globalCostmapData.info.width || check_y < 0 || check_y >= this->globalCostmapData.info.height)
        {
          continue;
        }

        // Get the index of the point in the costmap data
        int index = check_y * this->globalCostmapData.info.width + check_x;

        // If the map data is greater than 50, it is an obstacle
        if (this->globalCostmapData.data[index] >= 50)
        {
          ROS_INFO("Point is in obstacle. Regenerating a new random point");
          return true;
        }
      }
    }
  }

  // Return false if the point is not in an obstacle
  return false;
}

geometry_msgs::PoseStamped GoalPublisherNode::getRandomTargetInPackingArea()
{
  // x: 8.0 to 16.0, y: -6.25 to 1.0, yaw: -3.14 to 3.14
  double x, y, yaw;
  bool inObstacle = false;

  do
  {
    x = std::round((static_cast<double>(std::rand()) / RAND_MAX * 8.0 + 8) * 10) / 10.0;
    y = std::round((-6.25 + static_cast<double>(std::rand()) / RAND_MAX * 7.25) * 10) / 10.0;
    yaw = std::round((static_cast<double>(std::rand()) / RAND_MAX * 6.28 - 3.14) * 10) / 10.0;
    // Check if the point is in an obstacle
    inObstacle = isPointInObstacle(x, y);
  } while (inObstacle);

  std::cout << "Packing area dimensions: "
            << "x = " << x << ", "
            << "y = " << y << ", "
            << "yaw = " << yaw << std::endl;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  q.normalize();
  geometry_msgs::PoseStamped random_goal_in_packing_area;
  random_goal_in_packing_area.pose.position.x = x;
  random_goal_in_packing_area.pose.position.y = y;
  random_goal_in_packing_area.pose.orientation = tf2::toMsg(q);

  return random_goal_in_packing_area;
};

geometry_msgs::PoseStamped GoalPublisherNode::getGoalPoseFromConfig(const std::string& name)
{
  /**
   * Get the Transform from goal to world from the file
   */

  double x, y, yaw;
  nh_.getParam("/me5413_world" + name + "/x", x);
  nh_.getParam("/me5413_world" + name + "/y", y);
  nh_.getParam("/me5413_world" + name + "/yaw", yaw);
  nh_.getParam("/me5413_world/frame_id", this->world_frame_);

  std::cout << "Heading to " << name << ": "
            << "x = " << x << ", "
            << "y = " << y << ", "
            << "yaw = " << yaw << std::endl;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  q.normalize();

  geometry_msgs::PoseStamped P_world_goal;
  P_world_goal.pose.position.x = x;
  P_world_goal.pose.position.y = y;
  P_world_goal.pose.orientation = tf2::toMsg(q);

  return P_world_goal;
};

std::pair<double, double> GoalPublisherNode::calculatePoseError(const geometry_msgs::Pose& pose_robot, const geometry_msgs::Pose& pose_goal)
{
  // Positional Error
  const double position_error = std::sqrt(
      std::pow(pose_robot.position.x - pose_goal.position.x, 2) +
      std::pow(pose_robot.position.y - pose_goal.position.y, 2));

  // Heading Error
  tf2::Quaternion q_robot, q_goal;
  tf2::fromMsg(pose_robot.orientation, q_robot);
  tf2::fromMsg(pose_goal.orientation, q_goal);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

  double roll, pitch, yaw_robot, yaw_goal;
  m_robot.getRPY(roll, pitch, yaw_robot);
  m_goal.getRPY(roll, pitch, yaw_goal);

  const double heading_error = (yaw_robot - yaw_goal) / M_PI * 180.0;

  return std::pair<double, double>(position_error, heading_error);
}

}  // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_publisher_node");
  me5413_world::GoalPublisherNode goal_publisher_node;
  ros::spin();  // spin the ros node.
  return 0;
}