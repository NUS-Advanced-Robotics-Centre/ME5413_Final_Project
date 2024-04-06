  this->global_costmap_ = nav_msgs::OccupancyGrid();
  this->current_waypoint_index_ = 0;
  this->waypoints_ = createWaypoints();

    this->sub_global_costmap_ = nh_.subscribe("/move_base/global_costmap/costmap", 1, &BoxExplorerNode::globalCostmapCallback, this);

void BoxExplorerNode::updateCurrentWaypoint()
{
  // randomly select a waypoint
  current_waypoint_index_ = rand() % waypoints_.size();
  if (this->current_waypoint_index_ < this->waypoints_.size()) {
    geometry_msgs::PoseStamped goal_pose = this->waypoints_[this->current_waypoint_index_];
    if (!isPointInObstacle(goal_pose, this->global_costmap_)) {
      this->current_waypoint_index_++;
    }
  }
}

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

bool BoxExplorerNode::isPointInObstacle(const geometry_msgs::PoseStamped& Point, const nav_msgs::OccupancyGrid& costmap)
{
  float resolution = costmap.info.resolution;
  double x = Point.pose.position.x;
  double y = Point.pose.position.y;

  int map_x = static_cast<int>((x - costmap.info.origin.position.x) / resolution);
  int map_y = static_cast<int>((y - costmap.info.origin.position.y) / resolution);

  // Define the radius of the circle to check
  double cheakRadius = 0.3;
  int radiusCells = static_cast<int>(cheakRadius / resolution);

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
}