
#include <QWidget>
#include <QPushButton>

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_srvs/Trigger.h>

#include "waypoint_operator_panel.h"

namespace waypoint_visualization
{
    WaypointOperatorPanel::WaypointOperatorPanel(QWidget *parent)
        : rviz::Panel(parent),
          nh(),
          private_nh("~")
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

        switch_cancel_client = nh.serviceClient<std_srvs::Trigger>("waypoint_manager/waypoint_server/switch_cancel");
        next_waypoint_client = nh.serviceClient<std_srvs::Trigger>("waypoint_manager/waypoint_server/next_waypoint");
        waypoint_save_client = nh.serviceClient<std_srvs::Trigger>("waypoint_manager/waypoint_server/save");
        prev_waypoint_client = nh.serviceClient<std_srvs::Trigger>("waypoint_manager/waypoint_server/prev_waypoint");
    }

    WaypointOperatorPanel::~WaypointOperatorPanel()
    {
    }

    void WaypointOperatorPanel::callSwitchCancel()
    {
        ROS_INFO("Pushed callSwitchCancel()");
        std_srvs::Trigger trigger;
        switch_cancel_client.call(trigger);
    }

    void WaypointOperatorPanel::callNextWaypoint()
    {
        ROS_INFO("Pushed callNextWaypoint()");
        std_srvs::Trigger trigger;
        next_waypoint_client.call(trigger);
    }

    void WaypointOperatorPanel::callWaypointSave()
    {
        ROS_INFO("Pushed callWaypointSave()");
        std_srvs::Trigger trigger;
        waypoint_save_client.call(trigger);
    }

    void WaypointOperatorPanel::callPrevWaypoint()
    {
        ROS_INFO("Pushed callPrevWaypoint()");
        std_srvs::Trigger trigger;
        prev_waypoint_client.call(trigger);
    }

    void WaypointOperatorPanel::callSetGoalRadius()
    {
        ROS_INFO("Pushed callSetGoalWaypoint()");
        nh.setParam("/waypoint_manager/waypoint_visualization/set_goal_radius", getWaypointNumber());
    }

    float WaypointOperatorPanel::getWaypointNumber()
    {
        bool ok;
        float number = waypoint_number_input->text().toFloat(&ok);
        if (ok) {
            return number;
        } else {
            ROS_WARN("Invalid waypoint number entered");
            return -1.0f;
        }
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    waypoint_visualization::WaypointOperatorPanel,
    rviz::Panel)