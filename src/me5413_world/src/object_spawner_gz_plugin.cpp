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
  this->pub_factory_ = node->Advertise<msgs::Factory>("~/factory");
  clt_delete_objects_ = nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  this->sub_respawn_objects_ = nh_.subscribe("/rviz_panel/respawn_objects", 1, &ObjectSpawner::respawnCmdCallback, this);

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
  this->box_points.emplace_back(ignition::math::Vector2d(16.576, -5.96)); // add tree point

  for (int i = 1; i <= std::min(num, 9); ++i)
  {
    const std::string box_name = "number" + std::to_string(i);
    ignition::math::Vector2d point;
    bool has_collision = true;
    // Generate random box_points within a 10 by 8 area with a distance greater than 4.3
    while (has_collision)
    {
      has_collision = false;
      point = ignition::math::Vector2d(static_cast<double>(std::rand()) / RAND_MAX * 8.0 + 8,
                                       -6.25 + static_cast<double>(std::rand()) / RAND_MAX * 7.25);
      
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
    this->box_names.push_back(box_name);

    msgs::Factory box_msg;
    box_msg.set_sdf_filename("model://" + box_name); // TODO: change to our own file
    msgs::Set(box_msg.mutable_pose(),
              ignition::math::Pose3d(
              ignition::math::Vector3d(point.X(), point.Y(), 0.5),
              ignition::math::Quaterniond(0, 0, 0))
              );
    this->pub_factory_->Publish(box_msg);
  }

  // remove tree point
  // return box_points, box_names;

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

void ObjectSpawner::deleteObjects(const std::vector<std::string>& object_names)
{
  for (const auto& object_name : object_names)
  {
    deleteObject(object_name);
  }

  return;
};

void ObjectSpawner::respawnCmdCallback(const std_msgs::Int16::ConstPtr& respawn_msg)
{
  const int cmd = respawn_msg->data;
  if (cmd == 0)
  {
    deleteObject(this->cone_name);
    deleteObjects(this->box_names);
    ROS_INFO_STREAM("Random Objects Cleared!");
  }
  else if (cmd == 1)
  {
    deleteObject(this->cone_name);
    deleteObjects(this->box_names);
    spawnRandomCones();
    spawnRandomBoxes(9);
    ROS_INFO_STREAM("Random Objects Respawned!");
  }
  else
  {
    ROS_INFO_STREAM("Respawned Command Not Recognized!");
  }

  return;
};

} // namespace gazebo