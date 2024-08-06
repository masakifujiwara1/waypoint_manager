
#pragma once

#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLineEdit>

#include <ros/ros.h>
#include <rviz/panel.h>

namespace waypoint_visualization
{
    class WaypointOperatorPanel : public rviz::Panel
    {
        Q_OBJECT

    public:
        WaypointOperatorPanel(QWidget *parent = 0);
        virtual ~WaypointOperatorPanel();

    protected Q_SLOTS:
        void callSwitchCancel();
        void callNextWaypoint();
        void callWaypointSave();
        void callPrevWaypoint();
        void callSetGoalRadius();
        float getWaypointNumber();

    protected:
        QVBoxLayout *main_layout;
        QHBoxLayout *button_layout1,
            *button_layout2;

        QPushButton *switch_cancel_button,
            *next_waypoint_button,
            *waypoint_save_button,
            *prev_waypoint_button,
            *set_goal_radius_button;

        QLineEdit *waypoint_number_input;

        ros::NodeHandle nh,
            private_nh;

        ros::ServiceClient switch_cancel_client,
            next_waypoint_client,
            waypoint_save_client,
            prev_waypoint_client;
    };
}