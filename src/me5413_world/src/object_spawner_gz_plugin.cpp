/* object_spawner_gz_plugin.cpp

 * Copyright (C) 2024 nuslde, SS47816

 * Gazebo Plugin for spawning objects
 
**/

#include "me5413_world/object_spawner_gz_plugin.hpp"

namespace gazebo
{

ObjectSpawner::ObjectSpawner() : WorldPlugin() {};

ObjectSpawner::~ObjectSpawner() {};
  
void ObjectSpawner::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  // Create a new transport node
  transport::NodePtr node(new transport::Node());
  node->Init(_world->Name());
  clt_delete_objects_ = nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  this->timer_ = nh_.createTimer(ros::Duration(0.1), &ObjectSpawner::timerCallback, this);
  this->pub_factory_ = node->Advertise<msgs::Factory>("~/factory");
  this->sub_respawn_objects_ = nh_.subscribe("/rviz_panel/respawn_objects", 1, &ObjectSpawner::respawnCmdCallback, this);
  this->pub_rviz_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/gazebo/ground_truth/box_markers", 0);
  
  return;
};

void ObjectSpawner::timerCallback(const ros::TimerEvent&)
{
  // publish rviz markers
  this->pub_rviz_markers_.publish(this->box_markers_msg_);

  return;
};

void ObjectSpawner::spawnRandomCones()
{
  this->cone_name = "Construction Cone_0";

  msgs::Factory cone_msg;
  cone_msg.set_sdf_filename("model://construction_cone");

  std::srand(std::time(0));
  if (std::rand() % 2 == 0)
  {
    msgs::Set(cone_msg.mutable_pose(), ignition::math::Pose3d(
                ignition::math::Vector3d(12.7, 2.5, 0.1),
                ignition::math::Quaterniond(0, 0, 0)));
  }
  else
  {
    msgs::Set(cone_msg.mutable_pose(), ignition::math::Pose3d(
                ignition::math::Vector3d(16.9, 2.5, 0.1),
                ignition::math::Quaterniond(0, 0, 0)));
  }
  this->pub_factory_->Publish(cone_msg);
  
  return;
};

void ObjectSpawner::spawnRandomBoxes(const int num)
{
  if (num > 9)
  {
    ROS_WARN_STREAM("Maximum 9 boxes are allowed!, Spawning 9 boxes instead");
  }

  std::srand(std::time(0));
  this->box_names.clear();
  this->box_points.clear();
  this->box_markers_msg_.markers.clear();
  this->box_points.emplace_back(ignition::math::Vector3d(16.576, -5.96, 1.0)); // add tree point

  visualization_msgs::MarkerArray text_markers_msg;
  for (int i = 1; i <= std::min(num, 9); ++i)
  {
    ignition::math::Vector3d point;
    // Generate random box_points within a 10 by 8 area with a distance greater than 4.3
    bool has_collision = true;
    while (has_collision)
    {
      has_collision = false;
      point = ignition::math::Vector3d(static_cast<double>(std::rand()) / RAND_MAX * 8.0 + 8,
                                       -6.25 + static_cast<double>(std::rand()) / RAND_MAX * 7.25,
                                       0.4);
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
    const std::string box_name = "number" + std::to_string(i);
    this->box_names.push_back(box_name);

    msgs::Factory box_msg;
    box_msg.set_sdf_filename("model://" + box_name); // TODO: change to our own file
    // ignition::math::Vector3d spawn_point = ignition::math::Vector3d(point.X(), point.Y(), static_cast<double>(std::rand()) / RAND_MAX * 1.5 + 0.5);
    msgs::Set(box_msg.mutable_pose(), ignition::math::Pose3d(point, ignition::math::Quaterniond(0, 0, 0)));
    this->pub_factory_->Publish(box_msg);

    visualization_msgs::Marker box_marker;
    box_marker.header.frame_id = "world";
    box_marker.header.stamp = ros::Time();
    box_marker.ns = "gazebo";
    box_marker.id = i;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.action = visualization_msgs::Marker::ADD;
    box_marker.frame_locked = true;
    box_marker.lifetime = ros::Duration(0.2);
    box_marker.pose.position.x = point.X();
    box_marker.pose.position.y = point.Y();
    box_marker.pose.position.z = point.Z();
    box_marker.pose.orientation.x = 0.0;
    box_marker.pose.orientation.y = 0.0;
    box_marker.pose.orientation.z = 0.0;
    box_marker.pose.orientation.w = 1.0;
    box_marker.scale.x = 0.8;
    box_marker.scale.y = 0.8;
    box_marker.scale.z = 0.8;
    box_marker.color.a = 0.7;
    box_marker.color.r = static_cast<double>(std::rand()) / RAND_MAX * 0.5 + 0.25;
    box_marker.color.g = static_cast<double>(std::rand()) / RAND_MAX * 0.5 + 0.25;
    box_marker.color.b = static_cast<double>(std::rand()) / RAND_MAX * 0.5 + 0.25;
    this->box_markers_msg_.markers.emplace_back(box_marker);

    visualization_msgs::Marker text_marker = box_marker;
    text_marker.id = num + i;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.text = std::to_string(i);
    text_marker.pose.position.z += 0.5;
    text_marker.scale.z = 0.5;
    text_marker.color.a = 0.8;
    text_marker.color.r = 0.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 0.0;
    text_markers_msg.markers.emplace_back(text_marker);
  }

  // remove the tree point
  this->box_points.erase(this->box_points.begin());
  // merge the two marker arrays
  this->box_markers_msg_.markers.insert(this->box_markers_msg_.markers.end(), text_markers_msg.markers.begin(), text_markers_msg.markers.end());

  return;
};

void ObjectSpawner::deleteObject(const std::string& object_name)
{
  gazebo_msgs::DeleteModel delete_model_srv;
  delete_model_srv.request.model_name = object_name;
  this->clt_delete_objects_.call(delete_model_srv);
  if (delete_model_srv.response.success == true)
  {
    ROS_INFO_STREAM("Object: " << object_name << "successfully deleted");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to delete Object: " << object_name << std::endl);
  }

  return;
};

void ObjectSpawner::deleteCone()
{
  deleteObject(this->cone_name);
  this->cone_name = "";
  this->cone_point = ignition::math::Vector3d();

  return;
};

void ObjectSpawner::deleteBoxs()
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
    deleteCone();
    deleteBoxs();
    ROS_INFO_STREAM("Random Objects Cleared!");
  }
  else if (cmd == 1)
  {
    deleteCone();
    deleteBoxs();
    spawnRandomCones();
    spawnRandomBoxes(9);
    ROS_INFO_STREAM("Random Objects Respawned!");
  }
  else
  {
    ROS_INFO_STREAM("Respawn Command Not Recognized!");
  }

  return;
};

} // namespace gazebo