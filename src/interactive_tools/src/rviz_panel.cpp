#include <pluginlib/class_list_macros.hpp>
#include "interactive_tools/rviz_panel.hpp"

PLUGINLIB_EXPORT_CLASS(rviz_panel::simplePanel, rviz::Panel)

namespace rviz_panel
{
    simplePanel::simplePanel(QWidget * parent)
    :   rviz::Panel(parent),
        ui_(std::make_shared<Ui::two_button>())
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        // Define ROS publisher
        pub_goal_ = nh_.advertise<std_msgs::String>("/rviz_panel/goal_name", 1);
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

        // Initialization
        goal_name_msg_.data = "";
    }

    // Assembly Line buttons
    void simplePanel::on_button_1_1_clicked()
    {
        ROS_INFO_STREAM("Setting Assembly Line 1 as the GOAL.");
        ui_->label_status->setText("Heading to Assembly Line 1.");
        this->goal_name_msg_.data = "/assembly_line_1";
        this->pub_goal_.publish(goal_name_msg_);
    }
    void simplePanel::on_button_1_2_clicked()
    {
        ROS_INFO_STREAM("Setting Assembly Line 2 as the GOAL.");
        ui_->label_status->setText("Heading to Assembly Line 2.");
        this->goal_name_msg_.data = "/assembly_line_2";
        this->pub_goal_.publish(goal_name_msg_);
    }

    // Packaging Area buttons
    void simplePanel::on_button_2_1_clicked()
    {
        ROS_INFO_STREAM("Setting Packaging Area 1 as the GOAL.");
        ui_->label_status->setText("Heading to Packaging Area 1.");
        this->goal_name_msg_.data = "/packing_area_1";
        this->pub_goal_.publish(goal_name_msg_);
    }
    void simplePanel::on_button_2_2_clicked()
    {
        ROS_INFO_STREAM("Setting Packaging Area 2 as the GOAL.");
        ui_->label_status->setText("Heading to Packaging Area 2.");
        this->goal_name_msg_.data = "/packing_area_2";
        this->pub_goal_.publish(goal_name_msg_);
    }
    void simplePanel::on_button_2_3_clicked()
    {
        ROS_INFO_STREAM("Setting Packaging Area 3 as the GOAL.");
        ui_->label_status->setText("Heading to Packaging Area 3.");
        this->goal_name_msg_.data = "/packing_area_3";
        this->pub_goal_.publish(goal_name_msg_);
    }
    void simplePanel::on_button_2_4_clicked()
    {
        ROS_INFO_STREAM("Setting Packaging Area 4 as the GOAL.");
        ui_->label_status->setText("Heading to Packaging Area 4.");
        this->goal_name_msg_.data = "/packing_area_4";
        this->pub_goal_.publish(goal_name_msg_);
    }

    // Delivery Vehicle buttons
    void simplePanel::on_button_3_1_clicked()
    {
        ROS_INFO_STREAM("Setting Vehicle 1 as the GOAL.");
        ui_->label_status->setText("Heading to Vehicle 1.");
        this->goal_name_msg_.data = "/vehicle_1";
        this->pub_goal_.publish(goal_name_msg_);
    }
    void simplePanel::on_button_3_2_clicked()
    {
        ROS_INFO_STREAM("Setting Vehicle 2 as the GOAL.");
        ui_->label_status->setText("Heading to Vehicle 2.");
        this->goal_name_msg_.data = "/vehicle_2";
        this->pub_goal_.publish(goal_name_msg_);
    }
    void simplePanel::on_button_3_3_clicked()
    {
        ROS_INFO_STREAM("Setting Vehicle 3 as the GOAL.");
        ui_->label_status->setText("Heading to Vehicle 3.");
        this->goal_name_msg_.data = "/vehicle_3";
        this->pub_goal_.publish(goal_name_msg_);
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
