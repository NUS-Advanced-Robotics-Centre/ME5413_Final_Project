/* object_spawner_gz_plugin.cpp

 * Copyright (C) 2024 nuslde, SS47816

 * Gazebo Plugin for spawning objects
 
**/

#include <ctime>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <algorithm>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/DeleteModel.h>

namespace gazebo
{
class ObjectSpawner : public WorldPlugin
{
 public:
  std::string bridge_name;
  std::string cone_name;
  std::string random_cone_name;
  ignition::math::Vector3d bridge_point; //@shuo is this one still needed?
  std::vector<std::string> box_names;
  std::vector<ignition::math::Vector3d> box_points;

  ObjectSpawner();
  virtual ~ObjectSpawner();
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

 private:
  transport::PublisherPtr pub_factory_;
  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::ServiceClient clt_delete_objects_;
  ros::Subscriber sub_respawn_objects_;
  ros::Subscriber sub_cmd_open_bridge_;
  ros::Publisher pub_rviz_markers_;
  ros::Publisher pub_box_cmd_vel_;
  ros::Subscriber sub_box_odom_;

  visualization_msgs::MarkerArray box_markers_msg_;

  bool bridge_open_called_;
  double bridge_position_;
  
  void timerCallback(const ros::TimerEvent&);
  void spawnRandomBridge();  //deprecated for 2526
  void spawnRandomBoxes();
  void deleteObject(const std::string& object_name);
  void deleteBridge();  //not used for 2526
  void deleteCone();
  void deleteRandomCone();
  void spawnCone();
  void spawnRandomCone();
  void deleteBoxes();
  void respawnCmdCallback(const std_msgs::Int16::ConstPtr& respawn_msg);
  void openBridgeCallback(const std_msgs::Bool::ConstPtr& open_bridge_msg);
  void boxOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ObjectSpawner)

} // namespace gazebo