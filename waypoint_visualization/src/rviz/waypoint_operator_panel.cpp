
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
        hbox_layout = new QVBoxLayout();
        hbox_layout2 = new QVBoxLayout();
        box_layout = new QBoxLayout(QBoxLayout::RightToLeft);

        switch_cancel_button = new QPushButton(this);
        switch_cancel_button->setText("Switch Cancel");
        connect(switch_cancel_button, SIGNAL(clicked()), this, SLOT(callSwitchCancel()));
        switch_cancel_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        next_waypoint_button = new QPushButton(this);
        next_waypoint_button->setText("Next Waypoint");
        connect(next_waypoint_button, SIGNAL(clicked()), this, SLOT(callNextWaypoint()));
        next_waypoint_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        waypoint_save_button = new QPushButton(this);
        waypoint_save_button->setText("Waypoint Save");
        connect(waypoint_save_button, SIGNAL(clicked()), this, SLOT(callWaypointSave()));
        waypoint_save_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        prev_waypoint_button = new QPushButton(this);
        prev_waypoint_button->setText("Prev Waypoint");
        connect(prev_waypoint_button, SIGNAL(clicked()), this, SLOT(callPrevWaypoint()));
        prev_waypoint_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        hbox_layout2->addWidget(switch_cancel_button);
        hbox_layout->addWidget(next_waypoint_button);
        hbox_layout2->addWidget(waypoint_save_button);
        hbox_layout->addWidget(prev_waypoint_button);

        box_layout->addLayout(hbox_layout, 0);
        box_layout->addLayout(hbox_layout2, 0);
        setLayout(box_layout);

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
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    waypoint_visualization::WaypointOperatorPanel,
    rviz::Panel)