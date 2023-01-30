#ifndef PLANT_FLAG_TOOL_H
#define PLANT_FLAG_TOOL_H

#include <rviz/tool.h>

namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

namespace interactive_tools
{

class AddObjectTool: public rviz::Tool
{
Q_OBJECT
public:
  AddObjectTool();
  ~AddObjectTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

private:
  void makeFlag(const Ogre::Vector3& position);

  std::vector<Ogre::SceneNode*> flag_nodes_;
  Ogre::SceneNode* moving_flag_node_;
  std::string flag_resource_;
  rviz::VectorProperty* current_flag_property_;
};
} // end namespace interactive_tools

#endif // PLANT_FLAG_TOOL_H