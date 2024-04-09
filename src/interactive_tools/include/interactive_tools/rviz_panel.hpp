/* rviz_panel.hpp

 * Copyright (C) 2023 SS47816

 * Rviz Panel for controling goal poses

**/

#ifndef rviz_panel_H_
#define rviz_panel_H_

#include <ros/ros.h>
#include <rviz/panel.h>
#include <ui_simple_panel.h>
/**
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */

#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

namespace rviz_panel
{
class simplePanel : public rviz::Panel
{
  Q_OBJECT

 public:
  #ifdef UNIT_TEST
    friend class testClass;
  #endif
  /**
   *  QWidget subclass constructors usually take a parent widget
   *  parameter (which usually defaults to 0).  At the same time,
   *  pluginlib::ClassLoader creates instances by calling the default
   *  constructor (with no arguments). Taking the parameter and giving
   *  a default of 0 lets the default constructor work and also lets
   *  someone using the class for something else to pass in a parent
   *  widget as they normally would with Qt.
   */
  simplePanel(QWidget *parent = 0);

  /**
   *  Now we declare overrides of rviz::Panel functions for saving and
   *  loading data from the config file.  Here the data is the topic name.
   */
  virtual void save(rviz::Config config) const;
  virtual void load(const rviz::Config &config);

  public Q_SLOTS:
  /**
   *  Here we declare some internal slots.
   */
  private Q_SLOTS:
    // Assembly Line buttons
    void on_button_1_1_clicked();
    void on_button_1_2_clicked();
    // Packaging Area buttons
    void on_button_2_1_clicked();
    void on_button_2_2_clicked();
    void on_button_2_3_clicked();
    void on_button_2_4_clicked();
    // Delivery Vehicle buttons
    void on_button_3_1_clicked();
    void on_button_3_2_clicked();
    void on_button_3_3_clicked();
    // Contorl Buttons
    void on_button_regen_clicked();
    void on_button_clear_clicked();

 protected:
  // UI pointer
  std::shared_ptr<Ui::TaskControlPanel> ui_;
  // ROS declaration
  ros::NodeHandle nh_;
  ros::Publisher pub_goal_;
  ros::Publisher pub_respawn_;
  std_msgs::String goal_name_msg_;
  std_msgs::Int16 regen_cmd_msg_;
};

} // namespace rviz_panel

#endif