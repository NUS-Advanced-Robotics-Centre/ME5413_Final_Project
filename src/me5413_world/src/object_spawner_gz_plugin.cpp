/* object_spawner_gz_plugin.cpp

 * Copyright (C) 2024 nuslde, SS47816

 * Gazebo Plugin for spawning objects
 
**/

#include <ctime>
#include <cstdlib>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>

#include <ignition/math/Pose3.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
class ObjectSpawner : public WorldPlugin
{
 public:
  ObjectSpawner() : WorldPlugin() {};
  virtual ~ObjectSpawner() {};
  
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Create a new transport node
    transport::NodePtr node(new transport::Node());
    // Initialize the node with the world name
    node->Init(_world->Name());
    // Create a publisher on the ~/factory topic
    this->pub_factory = node->Advertise<msgs::Factory>("~/factory");
    
    this->sub_respawn_objects_ = nh_.subscribe("/me5413_world/respawn_objects", 1, &ObjectSpawner::respawnCmdCallback, this);
    
    spawnRandomObjects();
  }

 private:
  // Gazebo object messages
  msgs::Factory box_1_msg;
  msgs::Factory box_2_msg;
  msgs::Factory box_3_msg;
  msgs::Factory cone_msg;

  transport::PublisherPtr pub_factory;

  ros::NodeHandle nh_;
  ros::Subscriber sub_respawn_objects_;
  // ros::Publisher pub_rviz_markers_;

  void respawnCmdCallback(const std_msgs::Bool::ConstPtr& respawn_msg)
  {
    ROS_INFO_STREAM("Respawning Random Objects!");
    std::cout << "Respawning Random Objects!" << std::endl;

    spawnRandomObjects();

    ROS_INFO_STREAM("Random Objects Respawned!");
    std::cout << "Random Objects Respawned!" << std::endl;
    return;
  };

  void spawnRandomObjects()
  {
    /*********************************** Generate Random Boxes **********************************/
    // Seed for random number generation
    std::srand(std::time(0));
    // Generate random points within a 10 by 8 area with a distance greater than 4.3
    ignition::math::Vector2d p1, p2, p3;
    do
    {
      p1 = ignition::math::Vector2d(
        static_cast<double>(std::rand()) / RAND_MAX * 8.0 + 8,
        1 - static_cast<double>(std::rand()) / RAND_MAX * 7.25);

      p2 = ignition::math::Vector2d(
        static_cast<double>(std::rand()) / RAND_MAX * 8.0 + 8,
        -6.25 + static_cast<double>(std::rand()) / RAND_MAX * 7.25);

      p3 = ignition::math::Vector2d(
        static_cast<double>(std::rand()) / RAND_MAX * 8.0 + 8,
        -6.25 + static_cast<double>(std::rand()) / RAND_MAX * 7.25);
    } while ((p1 - p2).Length() <= 4.3 ||
              (p1 - p3).Length() <= 4.3 ||
              (p2 - p3).Length() <= 4.3 ||
              (p1.X() > 16.2 && p1.X() < 16.9 && p1.Y() > -6.16 && p1.Y() < -5.76) ||
              (p2.X() > 16.2 && p2.X() < 16.9 && p2.Y() > -6.16 && p2.Y() < -5.76) ||
              (p3.X() > 16.2 && p3.X() < 16.9 && p3.Y() > -6.16 && p3.Y() < -5.76));

    // Model file to load
    box_1_msg.set_sdf_filename("model://number1");
    box_2_msg.set_sdf_filename("model://number2");
    box_3_msg.set_sdf_filename("model://number3");
    cone_msg.set_sdf_filename("model://construction_cone");

    // Pose to initialize the model to
    msgs::Set(box_1_msg.mutable_pose(),
              ignition::math::Pose3d(
                  ignition::math::Vector3d(p1.X(), p1.Y(), 0.4),
                  ignition::math::Quaterniond(0, 0, 0)));

    msgs::Set(box_2_msg.mutable_pose(),
              ignition::math::Pose3d(
                  ignition::math::Vector3d(p2.X(), p2.Y(), 0.4),
                  ignition::math::Quaterniond(0, 0, 0)));

    msgs::Set(box_3_msg.mutable_pose(),
              ignition::math::Pose3d(
                  ignition::math::Vector3d(p3.X(), p3.Y(), 0.4),
                  ignition::math::Quaterniond(0, 0, 0)));

    /*********************************** Generate Traffic Cone **********************************/
    if (std::rand() % 2 == 0)
    {
      msgs::Set(cone_msg.mutable_pose(),
                ignition::math::Pose3d(
                    ignition::math::Vector3d(12.7, 2.5, 0.4),
                    ignition::math::Quaterniond(0, 0, 0)));
    }
    else
    {
      msgs::Set(cone_msg.mutable_pose(),
                ignition::math::Pose3d(
                    ignition::math::Vector3d(16.9, 2.5, 0.4),
                    ignition::math::Quaterniond(0, 0, 0)));
    }

    // Send messages
    pub_factory->Publish(box_1_msg);
    pub_factory->Publish(box_2_msg);
    pub_factory->Publish(box_3_msg);
    pub_factory->Publish(cone_msg);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ObjectSpawner)

} // namespace gazebo