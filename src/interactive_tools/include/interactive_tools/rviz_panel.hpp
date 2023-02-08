#ifndef rviz_panel_H_
#define rviz_panel_H_

#include <ros/ros.h>
#include <rviz/panel.h>

/** 
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_simple_panel.h>

// Other ROS dependencies
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rviz_panel
{
    /**
     *  Here we declare our new subclass of rviz::Panel. Every panel which
     *  can be added via the Panels/Add_New_Panel menu is a subclass of
     *  rviz::Panel.
     */

    class simplePanel : public rviz::Panel
    {
        /**
         * This class uses Qt slots and is a subclass of QObject, so it needs
         * the Q_OBJECT macro.
         */
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
            simplePanel(QWidget * parent = 0);

            /**
             *  Now we declare overrides of rviz::Panel functions for saving and
             *  loading data from the config file.  Here the data is the topic name.
             */
            virtual void save(rviz::Config config) const;
            virtual void load(const rviz::Config & config);

        /**
         *  Next come a couple of public Qt Slots.
         */
        public Q_SLOTS:

        /**
         *  Here we declare some internal slots.
         */
        private Q_SLOTS:

            void button_one();
            void button_two();
            geometry_msgs::PoseStamped getPoseMsgFromConfig(const std::string& name);
            void on_button_1_1_clicked();

        /**
         *  Finally, we close up with protected member variables
         */
        protected:
            // UI pointer
            std::shared_ptr<Ui::two_button> ui_;
            // ROS declaration
            ros::NodeHandle nh_;
            ros::Publisher button_1_pub_;
            ros::Publisher button_2_pub_;
            ros::Publisher pub_goal_;

            std_msgs::Bool msg_;
    };
} // namespace rviz_panel

#endif