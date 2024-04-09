/* rviz_panel.cpp

 * Copyright (C) 2023 SS47816

 * Rviz Panel for controling goal poses 
 
**/

#include <pluginlib/class_list_macros.hpp>
#include "interactive_tools/rviz_panel.hpp"

PLUGINLIB_EXPORT_CLASS(rviz_panel::simplePanel, rviz::Panel)

namespace rviz_panel
{
    simplePanel::simplePanel(QWidget * parent)
    :   rviz::Panel(parent),
        ui_(std::make_shared<Ui::TaskControlPanel>())
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        // Define ROS publisher
        this->pub_goal_ = nh_.advertise<std_msgs::String>("/rviz_panel/goal_name", 1);
        this->pub_respawn_ = nh_.advertise<std_msgs::Int16>("/rviz_panel/respawn_objects", 1);
        // sub_error_to_goal_ = nh_.subscribe("/interactive_tools/error_to_goal", 1, &GoalPublisherNode::goalPoseCallback, this);

        // Connect the clicked signals to slots
        connect(ui_->pushButton_1_1, SIGNAL(clicked()), this, SLOT(on_button_1_1_clicked()));
        connect(ui_->pushButton_1_2, SIGNAL(clicked()), this, SLOT(on_button_1_2_clicked()));

        connect(ui_->pushButton_2_1, SIGNAL(clicked()), this, SLOT(on_button_2_1_clicked()));
        connect(ui_->pushButton_2_2, SIGNAL(clicked()), this, SLOT(on_button_2_2_clicked()));
        connect(ui_->pushButton_2_3, SIGNAL(clicked()), this, SLOT(on_button_2_3_clicked()));
        connect(ui_->pushButton_2_4, SIGNAL(clicked()), this, SLOT(on_button_2_4_clicked()));

        connect(ui_->pushButton_3_1, SIGNAL(clicked()), this, SLOT(on_button_3_1_clicked()));
        connect(ui_->pushButton_3_2, SIGNAL(clicked()), this, SLOT(on_button_3_2_clicked()));
        connect(ui_->pushButton_3_3, SIGNAL(clicked()), this, SLOT(on_button_3_3_clicked()));

        connect(ui_->pushButton_regen, SIGNAL(clicked()), this, SLOT(on_button_regen_clicked()));
        connect(ui_->pushButton_clear, SIGNAL(clicked()), this, SLOT(on_button_clear_clicked()));

        // Initialization
        goal_name_msg_.data = "";
    }

    // Assembly Line buttons
    void simplePanel::on_button_1_1_clicked()
    {
        ROS_INFO_STREAM("Setting Assembly Line 1 as the GOAL.");
        ui_->label_status->setText("Heading to Assembly Line 1");
        this->goal_name_msg_.data = "/assembly_line_1";
        this->pub_goal_.publish(this->goal_name_msg_);
    }
    void simplePanel::on_button_1_2_clicked()
    {
        ROS_INFO_STREAM("Setting Assembly Line 2 as the GOAL.");
        ui_->label_status->setText("Heading to Assembly Line 2");
        this->goal_name_msg_.data = "/assembly_line_2";
        this->pub_goal_.publish(this->goal_name_msg_);
    }

    // Box buttons
    void simplePanel::on_button_2_1_clicked()
    {
        ROS_INFO_STREAM("Setting Box 1 as the GOAL.");
        ui_->label_status->setText("Heading to Box 1");
        this->goal_name_msg_.data = "/box_1";
        this->pub_goal_.publish(this->goal_name_msg_);
    }
    void simplePanel::on_button_2_2_clicked()
    {
        ROS_INFO_STREAM("Setting Box 2 as the GOAL.");
        ui_->label_status->setText("Heading to Box 2");
        this->goal_name_msg_.data = "/box_2";
        this->pub_goal_.publish(this->goal_name_msg_);
    }
    void simplePanel::on_button_2_3_clicked()
    {
        ROS_INFO_STREAM("Setting Box 3 as the GOAL.");
        ui_->label_status->setText("Heading to Box 3");
        this->goal_name_msg_.data = "/box_3";
        this->pub_goal_.publish(this->goal_name_msg_);
    }
    void simplePanel::on_button_2_4_clicked()
    {
        ROS_INFO_STREAM("Setting Box 4 as the GOAL.");
        ui_->label_status->setText("Heading to Box 4");
        this->goal_name_msg_.data = "/box_4";
        this->pub_goal_.publish(this->goal_name_msg_);
    }

    // Delivery Vehicle buttons
    void simplePanel::on_button_3_1_clicked()
    {
        ROS_INFO_STREAM("Setting Vehicle 1 as the GOAL.");
        ui_->label_status->setText("Heading to Vehicle 1");
        this->goal_name_msg_.data = "/vehicle_1";
        this->pub_goal_.publish(this->goal_name_msg_);
    }
    void simplePanel::on_button_3_2_clicked()
    {
        ROS_INFO_STREAM("Setting Vehicle 2 as the GOAL.");
        ui_->label_status->setText("Heading to Vehicle 2");
        this->goal_name_msg_.data = "/vehicle_2";
        this->pub_goal_.publish(this->goal_name_msg_);
    }
    void simplePanel::on_button_3_3_clicked()
    {
        ROS_INFO_STREAM("Setting Vehicle 3 as the GOAL.");
        ui_->label_status->setText("Heading to Vehicle 3");
        this->goal_name_msg_.data = "/vehicle_3";
        this->pub_goal_.publish(this->goal_name_msg_);
    }

    void simplePanel::on_button_regen_clicked()
    {
        ROS_INFO_STREAM("Respawning Random Objects");
        ui_->label_status->setText("Please select a goal pose");
        this->regen_cmd_msg_.data = 1;
        this->pub_respawn_.publish(this->regen_cmd_msg_);
    }
    void simplePanel::on_button_clear_clicked()
    {
        ROS_INFO_STREAM("Clearing Random Objects");
        ui_->label_status->setText("Please select a goal pose");
        this->regen_cmd_msg_.data = 0;
        this->pub_respawn_.publish(this->regen_cmd_msg_);
    }

    /**
     *  Save all configuration data from this panel to the given
     *  Config object. It is important here that you call save()
     *  on the parent class so the class id and panel name get saved.
     */
    void simplePanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    /**
     *  Load all configuration data for this panel from the given Config object.
     */
    void simplePanel::load(const rviz::Config & config)
    {
        rviz::Panel::load(config);
    }
} // namespace rviz_panel
