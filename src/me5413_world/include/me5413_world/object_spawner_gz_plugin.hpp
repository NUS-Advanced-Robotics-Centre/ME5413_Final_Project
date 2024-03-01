/* object_spawner_gz_plugin.cpp

 * Copyright (C) 2024 nuslde, SS47816

 * Gazebo Plugin for spawning objects
 
**/

#include <ctime>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int16.h>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/DeleteModel.h>

namespace gazebo
{
class ObjectSpawner : public WorldPlugin
{
 public:
  std::string cone_name;
  ignition::math::Vector2d cone_point;
  std::vector<std::string> box_names;
  std::vector<ignition::math::Vector2d> box_points;

  ObjectSpawner();
  virtual ~ObjectSpawner();
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

 private:
  transport::PublisherPtr pub_factory_;
  ros::NodeHandle nh_;
  ros::ServiceClient clt_delete_objects_;
  ros::Subscriber sub_respawn_objects_;
  ros::Publisher pub_rviz_markers_;
  
  void spawnRandomCones();
  void spawnRandomBoxes(const int num);
  void deleteObject(const std::string& object_name);
  void deleteObjects(const std::vector<std::string>& object_names);
  void respawnCmdCallback(const std_msgs::Int16::ConstPtr& respawn_msg);
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ObjectSpawner)

} // namespace gazebo