/* object_spawner_gz_plugin.cpp

 * Copyright (C) 2024 nuslde, SS47816

 * Gazebo Plugin for spawning objects
 
**/

#include "me5413_world/object_spawner_gz_plugin.hpp"

namespace gazebo
{

const int NUM_BOX_TYPES = 4;
const int MIN_X_COORD = 2.0;
const int MIN_Y_COORD = 11.0;
const int MAX_X_COORD = 22.0;
const int MAX_Y_COORD = 19.0;
const int Z_COORD = 3.0;

ObjectSpawner::ObjectSpawner() : WorldPlugin() {};

ObjectSpawner::~ObjectSpawner() {};
  
void ObjectSpawner::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  transport::NodePtr node(new transport::Node());
  node->Init(_world->Name());
  clt_delete_objects_ = nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  this->timer_ = nh_.createTimer(ros::Duration(0.1), &ObjectSpawner::timerCallback, this);
  this->pub_factory_ = node->Advertise<msgs::Factory>("~/factory");
  this->sub_respawn_objects_ = nh_.subscribe("/rviz_panel/respawn_objects", 1, &ObjectSpawner::respawnCmdCallback, this);
  this->sub_cmd_open_bridge_ = nh_.subscribe("/cmd_open_bridge", 1, &ObjectSpawner::openBridgeCallback, this);
  this->pub_rviz_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/gazebo/ground_truth/box_markers", 0);
  bridge_open_called_ = false;
  return;
};

void ObjectSpawner::timerCallback(const ros::TimerEvent&)
{
  // publish rviz markers
  this->pub_rviz_markers_.publish(this->box_markers_msg_);

  return;
};


void ObjectSpawner::spawnRandomBridge()
{
  msgs::Factory bridge_msg;
  this->bridge_name = "bridge";
  bridge_msg.set_sdf_filename("model://bridge");

  std::srand(std::time(0));
  bridge_position_ = (static_cast<double>(std::rand()) / RAND_MAX * 0.5 + 0.25) * (MAX_X_COORD - MIN_X_COORD) + MIN_X_COORD;
  msgs::Set(bridge_msg.mutable_pose(), ignition::math::Pose3d(
    ignition::math::Vector3d(bridge_position_, 9.0, 2.6), 
    ignition::math::Quaterniond(1.57079632679, 0, 0)));
  this->pub_factory_->Publish(bridge_msg);
  return;
};

void ObjectSpawner::spawnRandomBoxes()
{
  std::srand(std::time(0));
  this->box_names.clear();
  this->box_points.clear();
  this->box_markers_msg_.markers.clear();
  
  // The following two vectors should have the same size:
  std::vector<int> box_labels = {1, 2, 3, 4, 5, 6, 7, 8, 9}; // all possible box labels, between 1 and 9
  std::vector<int> box_nums = {1, 2, 3, 4, 5}; // can contain any positive number, but maek sure there's only one solution
  if (box_labels.size() < 1 || box_nums.size() < 1)
  {
    ROS_ERROR("The box_labels and box_nums should not be empty! Stoppping the spawning process");
    return;
  }

  // Randomise the number of boxes
  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(box_nums.begin(), box_nums.end(), g);
  std::shuffle(box_labels.begin(), box_labels.end(), g);
  box_nums = std::vector<int>(box_nums.begin(), box_nums.begin() + NUM_BOX_TYPES);
  box_labels = std::vector<int>(box_labels.begin(), box_labels.begin() + NUM_BOX_TYPES);
  
  std::vector<std::vector<int>> boxes;
  for (int i = 0; i < box_nums.size(); i++)
  {
    for (int j = 0; j < box_nums[i]; j++)
    {
      boxes.push_back(std::vector<int>{box_labels[i], j});
    }
  }

  // Generate destination box points
  const double spacing = (MAX_X_COORD - MIN_X_COORD)/(box_labels.size() + 1);
  for (int i = 0; i < box_labels.size(); i++)
  {
    const ignition::math::Vector3d point = ignition::math::Vector3d(spacing*(i + 1) + MIN_X_COORD, 0.0, Z_COORD);
    msgs::Factory box_msg;
    const std::string box_name = "number" + std::to_string(box_labels[i]);
    this->box_names.push_back(box_name);
    box_msg.set_sdf_filename("model://" + box_name);
    msgs::Set(box_msg.mutable_pose(), ignition::math::Pose3d(point, ignition::math::Quaterniond(0, 0, 0)));
    this->pub_factory_->Publish(box_msg);
    ROS_DEBUG_STREAM("Generated " << box_name << " at " << point);
    common::Time::MSleep(500);
  }

  // Generate random box points
  // visualization_msgs::MarkerArray text_markers_msg;
  for (int i = 0; i < boxes.size(); i++)
  {
    ignition::math::Vector3d point;
    bool has_collision = true;
    // Check for collsions
    while (has_collision)
    {
      has_collision = false;
      point = ignition::math::Vector3d(static_cast<double>(std::rand()) / RAND_MAX * (MAX_X_COORD - MIN_X_COORD) + MIN_X_COORD,
                                       static_cast<double>(std::rand()) / RAND_MAX * (MAX_Y_COORD - MIN_Y_COORD) + MIN_Y_COORD,
                                       Z_COORD);
      for (const auto& pre_point : this->box_points)
      {
        const double dist = (point - pre_point).Length();
        if (dist <= 1.2)
        {
          has_collision = true;
          break;
        }
      }
    } 

    // Add this box to the list
    this->box_points.push_back(point);
    
    // Publish gazebo model for this box
    msgs::Factory box_msg;
    const std::string box_name = "number" + std::to_string(boxes[i][0]);
    box_msg.set_sdf_filename("model://" + box_name);
    this->box_names.push_back("number" + std::to_string(boxes[i][0]) + "_" + std::to_string(boxes[i][1]));
    msgs::Set(box_msg.mutable_pose(), ignition::math::Pose3d(point, ignition::math::Quaterniond(0, 0, 0)));
    this->pub_factory_->Publish(box_msg);
    ROS_DEBUG_STREAM("Generated " << box_name << " at " << point);
    common::Time::MSleep(500);
    // // Publish rviz marker for this box
    // visualization_msgs::Marker box_marker;
    // box_marker.header.frame_id = "world";
    // box_marker.header.stamp = ros::Time();
    // box_marker.ns = "gazebo";
    // box_marker.id = 2*i;
    // box_marker.type = visualization_msgs::Marker::CUBE;
    // box_marker.action = visualization_msgs::Marker::ADD;
    // box_marker.frame_locked = true;
    // box_marker.lifetime = ros::Duration(0.2);
    // box_marker.pose.position.x = point.X();
    // box_marker.pose.position.y = point.Y();
    // box_marker.pose.position.z = point.Z();
    // box_marker.pose.orientation.x = 0.0;
    // box_marker.pose.orientation.y = 0.0;
    // box_marker.pose.orientation.z = 0.0;
    // box_marker.pose.orientation.w = 1.0;
    // box_marker.scale.x = 0.8;
    // box_marker.scale.y = 0.8;
    // box_marker.scale.z = 0.8;
    // box_marker.color.a = 0.7;
    // box_marker.color.r = static_cast<double>(std::rand()) / RAND_MAX * 0.5 + 0.25;
    // box_marker.color.g = static_cast<double>(std::rand()) / RAND_MAX * 0.5 + 0.25;
    // box_marker.color.b = static_cast<double>(std::rand()) / RAND_MAX * 0.5 + 0.25;
    // this->box_markers_msg_.markers.emplace_back(box_marker);

    // visualization_msgs::Marker text_marker = box_marker;
    // text_marker.id = 2*i + 1;
    // text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // text_marker.text = std::to_string(boxes[i][0]);
    // text_marker.pose.position.z += 0.5;
    // text_marker.scale.z = 0.5;
    // text_marker.color.a = 0.8;
    // text_marker.color.r = 0.0;
    // text_marker.color.g = 0.0;
    // text_marker.color.b = 0.0;
    // text_markers_msg.markers.emplace_back(text_marker);
  }

  // // merge the two marker arrays
  // this->box_markers_msg_.markers.insert(this->box_markers_msg_.markers.end(), text_markers_msg.markers.begin(), text_markers_msg.markers.end());

  return;
};

void ObjectSpawner::deleteObject(const std::string& object_name)
{
  gazebo_msgs::DeleteModel delete_model_srv;
  delete_model_srv.request.model_name = object_name;
  this->clt_delete_objects_.call(delete_model_srv);
  if (delete_model_srv.response.success == true)
  {
    ROS_INFO_STREAM("Object: " << object_name << " successfully deleted");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to delete Object: " << object_name << std::endl);
  }

  return;
};

void ObjectSpawner::deleteBridge()
{
  deleteObject(this->bridge_name);
  this->bridge_name = "";
  this->bridge_point = ignition::math::Vector3d();

  return;
};

void ObjectSpawner::spawnCone()
{
  msgs::Factory cone_msg;
  this->cone_name = "Construction Barrel";
  cone_msg.set_sdf_filename("model://construction_barrel");

  msgs::Set(cone_msg.mutable_pose(), ignition::math::Pose3d(
    ignition::math::Vector3d(bridge_position_ + 0.8, 7.0, 3.0), //centre of bridge
    ignition::math::Quaterniond(0, 0, 0)));
  this->pub_factory_->Publish(cone_msg);

  return;
};

void ObjectSpawner::deleteCone()
{
  deleteObject(this->cone_name);
  this->cone_name = "";

  return;
};


void ObjectSpawner::deleteBoxes()
{
  this->box_markers_msg_.markers.clear();
  this->pub_rviz_markers_.publish(this->box_markers_msg_);

  for (const auto& box_name: this->box_names)
  {
    deleteObject(box_name);
  }
  this->box_names.clear();
  this->box_points.clear();

  return;
};

void ObjectSpawner::respawnCmdCallback(const std_msgs::Int16::ConstPtr& respawn_msg)
{
  const int cmd = respawn_msg->data;
  if (cmd == 0)
  {
    deleteBridge();
    deleteCone();
    deleteBoxes();
    ROS_INFO_STREAM("Random Objects Cleared!");
  }
  else if (cmd == 1)
  {
    deleteCone();
    deleteBridge();
    deleteBoxes();
    spawnRandomBridge();
    spawnRandomBoxes();
    spawnCone();
    ROS_INFO_STREAM("Random Objects Respawned!");
    bridge_open_called_ = false;
  }
  else
  {
    ROS_INFO_STREAM("Respawn Command Not Recognized!");
  }

  return;
};

void ObjectSpawner::openBridgeCallback(const std_msgs::Bool::ConstPtr& open_bridge_msg)
{
  const bool open_bridge = open_bridge_msg->data;
  if (open_bridge == true)
  {
    if (bridge_open_called_ == false)
    {
      bridge_open_called_ = true;
      deleteCone();
      ROS_INFO_STREAM("Bridge will now open for 10s");
      common::Time::Sleep(10);
      spawnCone();
      ROS_INFO_STREAM("Bridge is now closed, cannot be opened again");
    }
    else
    {
      ROS_INFO_STREAM("Bridge has been opened before, cannot be opened again");
    }
  }
  else
  {
    ROS_INFO_STREAM("Bridge open command is false, nothing to be done");
  }
}

} // namespace gazebo