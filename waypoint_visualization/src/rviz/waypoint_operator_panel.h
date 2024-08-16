#pragma once

#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLineEdit>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace waypoint_visualization
{
    class WaypointOperatorPanel : public rviz_common::Panel
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
        QHBoxLayout *button_layout1, *button_layout2;

        QPushButton *switch_cancel_button,
                    *next_waypoint_button,
                    *waypoint_save_button,
                    *prev_waypoint_button,
                    *set_goal_radius_button;

        QLineEdit *waypoint_number_input;

        // ROS2ノード
        rclcpp::Node::SharedPtr node_;

        // ROS2サービスクライアント
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr switch_cancel_client,
                                                          next_waypoint_client,
                                                          waypoint_save_client,
                                                          prev_waypoint_client;
    };
}
