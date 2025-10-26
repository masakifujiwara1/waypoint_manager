
#include <QWidget>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "waypoint_operator_panel.h"

namespace waypoint_visualization
{
    WaypointOperatorPanel::WaypointOperatorPanel(QWidget *parent)
        : rviz_common::Panel(parent)
    {
        main_layout = new QVBoxLayout();
        button_layout1 = new QHBoxLayout();
        button_layout2 = new QHBoxLayout();

        waypoint_number_input = new QLineEdit(this);
        waypoint_number_input->setPlaceholderText("Enter waypoint number");
        main_layout->addWidget(waypoint_number_input);

        set_goal_radius_button = new QPushButton(this);
        set_goal_radius_button->setText("Set Goal Radius");
        connect(set_goal_radius_button, SIGNAL(clicked()), this, SLOT(callSetGoalRadius()));
        set_goal_radius_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        main_layout->addWidget(set_goal_radius_button);

        switch_cancel_button = new QPushButton(this);
        switch_cancel_button->setText("Switch Cancel");
        connect(switch_cancel_button, SIGNAL(clicked()), this, SLOT(callSwitchCancel()));
        switch_cancel_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        button_layout1->addWidget(switch_cancel_button);

        next_waypoint_button = new QPushButton(this);
        next_waypoint_button->setText("Next Waypoint");
        connect(next_waypoint_button, SIGNAL(clicked()), this, SLOT(callNextWaypoint()));
        next_waypoint_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        button_layout1->addWidget(next_waypoint_button);

        waypoint_save_button = new QPushButton(this);
        waypoint_save_button->setText("Waypoint Save");
        connect(waypoint_save_button, SIGNAL(clicked()), this, SLOT(callWaypointSave()));
        waypoint_save_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        button_layout2->addWidget(waypoint_save_button);

        prev_waypoint_button = new QPushButton(this);
        prev_waypoint_button->setText("Prev Waypoint");
        connect(prev_waypoint_button, SIGNAL(clicked()), this, SLOT(callPrevWaypoint()));
        prev_waypoint_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        button_layout2->addWidget(prev_waypoint_button);

        main_layout->addLayout(button_layout1);
        main_layout->addLayout(button_layout2);
        setLayout(main_layout);
    }

    WaypointOperatorPanel::~WaypointOperatorPanel()
    {
    }

    void WaypointOperatorPanel::onInitialize()
    {
        node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        switch_cancel_client_ = node_->create_client<std_srvs::srv::Trigger>(
            "waypoint_manager/waypoint_server/switch_cancel");
        next_waypoint_client_ = node_->create_client<std_srvs::srv::Trigger>(
            "waypoint_manager/waypoint_server/next_waypoint");
        waypoint_save_client_ = node_->create_client<std_srvs::srv::Trigger>(
            "waypoint_manager/waypoint_server/save");
        prev_waypoint_client_ = node_->create_client<std_srvs::srv::Trigger>(
            "waypoint_manager/waypoint_server/prev_waypoint");
    }

    void WaypointOperatorPanel::callSwitchCancel()
    {
        RCLCPP_INFO(node_->get_logger(), "Pushed callSwitchCancel()");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        switch_cancel_client_->async_send_request(request);
    }

    void WaypointOperatorPanel::callNextWaypoint()
    {
        RCLCPP_INFO(node_->get_logger(), "Pushed callNextWaypoint()");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        next_waypoint_client_->async_send_request(request);
    }

    void WaypointOperatorPanel::callWaypointSave()
    {
        RCLCPP_INFO(node_->get_logger(), "Pushed callWaypointSave()");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        waypoint_save_client_->async_send_request(request);
    }

    void WaypointOperatorPanel::callPrevWaypoint()
    {
        RCLCPP_INFO(node_->get_logger(), "Pushed callPrevWaypoint()");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        prev_waypoint_client_->async_send_request(request);
    }

    void WaypointOperatorPanel::callSetGoalRadius()
    {
        RCLCPP_INFO(node_->get_logger(), "Pushed callSetGoalRadius()");
        float radius = getWaypointNumber();
        
        if (radius < 0.0f) {
            return;  // Invalid input
        }
        
        // Create parameter client for waypoint_visualization node
        auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(
            node_, "waypoint_manager/waypoint_visualization");
        
        // Wait for the parameter service to be available
        if (!param_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(), "Parameter service not available");
            return;
        }
        
        // Set the parameter
        auto results = param_client->set_parameters({
            rclcpp::Parameter("set_goal_radius", radius)
        });
        
        RCLCPP_INFO(node_->get_logger(), "Set goal radius to: %f", radius);
    }

    float WaypointOperatorPanel::getWaypointNumber()
    {
        bool ok;
        float number = waypoint_number_input->text().toFloat(&ok);
        if (ok) {
            return number;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Invalid waypoint number entered");
            return -1.0f;
        }
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    waypoint_visualization::WaypointOperatorPanel,
    rviz_common::Panel)