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
    // Option 1: Insert model from file via function call.
    // The filename must be in the GAZEBO_MODEL_PATH environment variable.
    //_parent->InsertModelFile("model://box");

    // Option 2: Insert model from string via function call.
    // Insert a sphere model from string

    // sdf::SDF sphereSDF;
    // sphereSDF.SetFromString(
    //    "<sdf version ='1.4'>\
    //       <model name ='sphere'>\
    //         <pose>10 0 0 0 0 0</pose>\
    //         <link name ='link'>\
    //           <pose>0 0 .5 0 0 0</pose>\
    //           <collision name ='collision'>\
    //             <geometry>\
    //               <sphere><radius>0.5</radius></sphere>\
    //             </geometry>\
    //           </collision>\
    //           <visual name ='visual'>\
    //             <geometry>\
    //               <sphere><radius>0.5</radius></sphere>\
    //             </geometry>\
    //           </visual>\
    //         </link>\
    //       </model>\
    //     </sdf>");
    // // Demonstrate using a custom model name.
    // sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
    // model->GetAttribute("name")->SetFromString("unique_sphere");
    // _parent->InsertModelSDF(sphereSDF);

    // Option 3: Insert model from file via message passing.
    {
      // Create a new transport node
      transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      node->Init(_parent->Name());

      // Create a publisher on the ~/factory topic
      transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>("~/factory");

      // Create the message
      msgs::Factory msg_1;
      msgs::Factory msg_2;
      msgs::Factory msg_3;
      msgs::Factory msg_cone;

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
      msg_1.set_sdf_filename("model://number1");
      msg_2.set_sdf_filename("model://number2");
      msg_3.set_sdf_filename("model://number3");
      msg_cone.set_sdf_filename("model://construction_cone");

      // Seed for random number generation
      std::srand(std::time(0));

      if (std::rand() % 2 == 0)
      {
        msgs::Set(msg_cone.mutable_pose(),
                  ignition::math::Pose3d(
                      ignition::math::Vector3d(12.7, 2.5, 0.4),
                      ignition::math::Quaterniond(0, 0, 0)));
      }
      else
      {
        msgs::Set(msg_cone.mutable_pose(),
                  ignition::math::Pose3d(
                      ignition::math::Vector3d(16.9, 2.5, 0.4),
                      ignition::math::Quaterniond(0, 0, 0)));
      }

      // Pose to initialize the model to
      msgs::Set(msg_1.mutable_pose(),
                ignition::math::Pose3d(
                    ignition::math::Vector3d(p1.X(), p1.Y(), 0.4),
                    ignition::math::Quaterniond(0, 0, 0)));

      msgs::Set(msg_2.mutable_pose(),
                ignition::math::Pose3d(
                    ignition::math::Vector3d(p2.X(), p2.Y(), 0.4),
                    ignition::math::Quaterniond(0, 0, 0)));

      msgs::Set(msg_3.mutable_pose(),
                ignition::math::Pose3d(
                    ignition::math::Vector3d(p3.X(), p3.Y(), 0.4),
                    ignition::math::Quaterniond(0, 0, 0)));

      // Send the message
      factoryPub->Publish(msg_1);
      factoryPub->Publish(msg_2);
      factoryPub->Publish(msg_3);
      factoryPub->Publish(msg_cone);
    }
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ObjectSpawner)

} // namespace gazebo