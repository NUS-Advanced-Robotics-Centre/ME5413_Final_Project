#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <cstdlib>
#include <ctime>

namespace gazebo
{
class ObjectSpawner : public WorldPlugin
{
public:
  void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Create a new transport node
    transport::NodePtr node(new transport::Node());

    // Initialize the node with the world name
    node->Init(_parent->Name());

    // Create a publisher on the ~/factory topic
    transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>("~/factory");

    // Create the message
    msgs::Factory box_1_msg;
    msgs::Factory box_2_msg;
    msgs::Factory box_3_msg;
    msgs::Factory cone_msg;

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
    factoryPub->Publish(box_1_msg);
    factoryPub->Publish(box_2_msg);
    factoryPub->Publish(box_3_msg);
    factoryPub->Publish(cone_msg);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ObjectSpawner)

} // namespace gazebo