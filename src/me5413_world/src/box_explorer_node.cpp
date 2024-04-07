#include "me5413_world/box_explorer_node.hpp"

namespace me5413_world 
{

BoxExplorerNode::BoxExplorerNode() : tf2_listener_(tf2_buffer_)
{
  this->pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  this->sub_goal_name_ = nh_.subscribe("/rviz_panel/goal_name", 1, &BoxExplorerNode::goalNameCallback, this);
  this->sub_box_markers_ = nh_.subscribe("/gazebo/ground_truth/box_markers", 1, &BoxExplorerNode::boxMarkersCallback, this);
  this->sub_global_costmap_ = nh_.subscribe("/move_base/global_costmap/costmap", 1, &BoxExplorerNode::globalCostmapCallback, this);
  this->sub_goal_pose_detected_ = nh_.subscribe("/detected_goal_pose", 1, &BoxExplorerNode::goalPoseDetectedCallback, this);
  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &BoxExplorerNode::robotOdomCallback, this);

  // Initialization
  this->robot_frame_ = "base_link";
  this->map_frame_ = "map";
  this->world_frame_ = "world";
  this->goal_box_id_ = 0;

  this->global_costmap_ = nav_msgs::OccupancyGrid();
  this->current_waypoint_index_ = 0;
  this->waypoints_ = createWaypoints();

  this->is_goal_detected_ = false;
};

void BoxExplorerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->pose_world_robot_ = odom->pose.pose;

  updateGoalIfReached();
  return;
};

void BoxExplorerNode::goalNameCallback(const std_msgs::String::ConstPtr& name)
{ 
  const std::string goal_name = name->data;
  const int end = goal_name.find_last_of("_");
  this->goal_type_ = goal_name.substr(1, end-1);
  const int goal_box_id = stoi(goal_name.substr(end+1, 1));

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

    // P_world_goal = box_poses_[goal_box_id];
    // updateCurrentWaypoint();
    this->current_waypoint_index_ = rand() % waypoints_.size();
    P_world_goal = waypoints_[this->current_waypoint_index_];
  }
  else
  {
    this->goal_box_id_ = 0;
    return;
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

  // Publish goal pose in map frame on it goal type is box
  if (this->goal_type_ == "box")
  {
    this->pub_goal_.publish(P_map_goal);
  }

  return;
};

void BoxExplorerNode::boxMarkersCallback(const visualization_msgs::MarkerArray::ConstPtr& box_markers)
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

void BoxExplorerNode::globalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap)
{
  this->global_costmap_ = *costmap;
  return;
};

void BoxExplorerNode::goalPoseDetectedCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose)
{
  this->is_goal_detected_ = true;

  geometry_msgs::PoseStamped P_world_goal;
  if (this->goal_type_ == "box")
  {
    P_world_goal = *goal_pose;
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

    // Publish goal pose in map frame on it goal type is box
    this->pub_goal_.publish(P_map_goal);
  }
  return;
};

std::vector<geometry_msgs::PoseStamped> BoxExplorerNode::createWaypoints()
{
  // Define the dimensions of the packing area
  double x_min = 8;
  double x_max = 16;
  double y_min = -6.25;
  double y_max = 1.0;
  double yaw_min = 3.14;
  double yaw_max = 6.28;

  // Define the grid size or waypoint spacing
  double grid_size = 1.0;

  // Create a list of waypoints or grid cells to explore
  std::vector<geometry_msgs::PoseStamped> waypoints;
  for (double x = x_min; x <= x_max; x += grid_size)
  {
    for (double y = y_min; y <= y_max; y += grid_size)
    {
      geometry_msgs::PoseStamped waypoint;
      double rand_yaw = std::round((static_cast<double>(std::rand()) / RAND_MAX * 6.28 - 3.14) * 10) / 10.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, rand_yaw);
      q.normalize();
      waypoint.pose.position.x = x;
      waypoint.pose.position.y = y;
      waypoint.pose.orientation = tf2::toMsg(q);
      waypoints.push_back(waypoint);
    }
  }

  return waypoints;
}

void BoxExplorerNode::updateCurrentWaypoint()
{
  // randomly select a waypoint
  current_waypoint_index_ = rand() % waypoints_.size();
  if (this->current_waypoint_index_ < this->waypoints_.size()) {
    geometry_msgs::PoseStamped goal_pose = this->waypoints_[this->current_waypoint_index_];
    while (!isPointInObstacle(goal_pose, this->global_costmap_)) {
      this->current_waypoint_index_ = rand() % waypoints_.size();
    }
  }
}

bool BoxExplorerNode::isPointInObstacle(const geometry_msgs::PoseStamped& Point, const nav_msgs::OccupancyGrid& costmap)
{
  float resolution = costmap.info.resolution;
  double x = Point.pose.position.x;
  double y = Point.pose.position.y;

  int map_x = static_cast<int>((x - costmap.info.origin.position.x) / resolution);
  int map_y = static_cast<int>((y - costmap.info.origin.position.y) / resolution);

  // Define the radius of the circle to check
  double checkRadius = 0.3;
  int radiusCells = static_cast<int>(checkRadius / resolution);

  for (int dx = -radiusCells; dx <= radiusCells; ++dx) {
    for (int dy = -radiusCells; dy <= radiusCells; ++dy) {
      int check_x = map_x + dx;
      int check_y = map_y + dy;

      // Ensure the point is within the circle
      if (dx*dx + dy*dy <= radiusCells*radiusCells) {
        // Ignore points outside the map
        if (check_x < 0 || check_x >= costmap.info.width ||
            check_y < 0 || check_y >= costmap.info.height) {
          continue;
        }

        // Get the index of the point in the costmap
        int index = check_y * costmap.info.width + check_x;

        // If the map data is greater than or equal to 50, it is an obstacle
        if (costmap.data[index] >= 50) {
          ROS_INFO("Point is in obstacle...");
          return true;
        }
      }
    }
  }

  // If no obstacles are found, return false
  return false;
}

void BoxExplorerNode::updateGoalIfReached()
{
  // Check if the robot has reached the current goal
  double distance = std::sqrt(std::pow(this->pose_world_robot_.position.x - this->pose_world_goal_.position.x, 2) +
                              std::pow(this->pose_world_robot_.position.y - this->pose_world_goal_.position.y, 2));

  // If the robot is within a certain threshold of the goal and the goal is not detected
  if (distance < 0.02 && !this->is_goal_detected_&& this->goal_type_ == "box")
  {
    ROS_INFO("Goal reached but not detected. Updating goal...");
    
    // Update the current waypoint index
    this->current_waypoint_index_ = (this->current_waypoint_index_ + 1) % this->waypoints_.size();
    
    // Get the new goal pose
    geometry_msgs::PoseStamped P_world_goal = this->waypoints_[this->current_waypoint_index_];
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

    // Publish the new goal pose in map frame
    this->pub_goal_.publish(P_map_goal);
  }
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_publisher_node");
  me5413_world::BoxExplorerNode goal_publisher_node;
  ros::spin();  // spin the ros node.
  return 0;
}