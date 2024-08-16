#include <QWidget>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "waypoint_operator_panel.hpp"

namespace waypoint_visualization
{
    WaypointOperatorPanel::WaypointOperatorPanel(QWidget *parent)
        : rviz_common::Panel(parent),
          node_(rclcpp::Node::make_shared("waypoint_operator_panel"))
    {
        main_layout = new QVBoxLayout();
        button_layout1 = new QHBoxLayout();
        button_layout2 = new QHBoxLayout();

        waypoint_number_input = new QLineEdit(this);
        waypoint_number_input->setPlaceholderText("Enter waypoint number");
        main_layout->addWidget(waypoint_number_input);

        set_goal_radius_button = new QPushButton(this);
        set_goal_radius_button->setText("Set Goal Radius");
        connect(set_goal_radius_button, &QPushButton::clicked, this, &WaypointOperatorPanel::callSetGoalRadius);
        set_goal_radius_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        main_layout->addWidget(set_goal_radius_button);

        switch_cancel_button = new QPushButton(this);
        switch_cancel_button->setText("Switch Cancel");
        connect(switch_cancel_button, &QPushButton::clicked, this, &WaypointOperatorPanel::callSwitchCancel);
        switch_cancel_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        button_layout1->addWidget(switch_cancel_button);

        next_waypoint_button = new QPushButton(this);
        next_waypoint_button->setText("Next Waypoint");
        connect(next_waypoint_button, &QPushButton::clicked, this, &WaypointOperatorPanel::callNextWaypoint);
        next_waypoint_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        button_layout1->addWidget(next_waypoint_button);

        waypoint_save_button = new QPushButton(this);
        waypoint_save_button->setText("Waypoint Save");
        connect(waypoint_save_button, &QPushButton::clicked, this, &WaypointOperatorPanel::callWaypointSave);
        waypoint_save_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        button_layout2->addWidget(waypoint_save_button);

        prev_waypoint_button = new QPushButton(this);
        prev_waypoint_button->setText("Prev Waypoint");
        connect(prev_waypoint_button, &QPushButton::clicked, this, &WaypointOperatorPanel::callPrevWaypoint);
        prev_waypoint_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        button_layout2->addWidget(prev_waypoint_button);

        main_layout->addLayout(button_layout1);
        main_layout->addLayout(button_layout2);
        setLayout(main_layout);

        switch_cancel_client = node_->create_client<std_srvs::srv::Trigger>("waypoint_manager/waypoint_server/switch_cancel");
        next_waypoint_client = node_->create_client<std_srvs::srv::Trigger>("waypoint_manager/waypoint_server/next_waypoint");
        waypoint_save_client = node_->create_client<std_srvs::srv::Trigger>("waypoint_manager/waypoint_server/save");
        prev_waypoint_client = node_->create_client<std_srvs::srv::Trigger>("waypoint_manager/waypoint_server/prev_waypoint");
    }

    WaypointOperatorPanel::~WaypointOperatorPanel()
    {
    }

    void WaypointOperatorPanel::callSwitchCancel()
    {
        RCLCPP_INFO(node_->get_logger(), "Pushed callSwitchCancel()");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        if (!switch_cancel_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(), "Service not available: switch_cancel");
            return;
        }
        switch_cancel_client->async_send_request(request);
    }

    void WaypointOperatorPanel::callNextWaypoint()
    {
        RCLCPP_INFO(node_->get_logger(), "Pushed callNextWaypoint()");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        if (!next_waypoint_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(), "Service not available: next_waypoint");
            return;
        }
        next_waypoint_client->async_send_request(request);
    }

    void WaypointOperatorPanel::callWaypointSave()
    {
        RCLCPP_INFO(node_->get_logger(), "Pushed callWaypointSave()");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        if (!waypoint_save_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(), "Service not available: waypoint_save");
            return;
        }
        waypoint_save_client->async_send_request(request);
    }

    void WaypointOperatorPanel::callPrevWaypoint()
    {
        RCLCPP_INFO(node_->get_logger(), "Pushed callPrevWaypoint()");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        if (!prev_waypoint_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(), "Service not available: prev_waypoint");
            return;
        }
        prev_waypoint_client->async_send_request(request);
    }

    void WaypointOperatorPanel::callSetGoalRadius()
    {
        RCLCPP_INFO(node_->get_logger(), "Pushed callSetGoalRadius()");
        node_->set_parameter(rclcpp::Parameter("/waypoint_manager/waypoint_visualization/set_goal_radius", getWaypointNumber()));
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
PLUGINLIB_EXPORT_CLASS(waypoint_visualization::WaypointOperatorPanel, rviz_common::Panel)
