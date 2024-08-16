#include <limits>
#include <atomic>
#include <string>
#include <stdexcept>
#include <exception>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "std_msgs/msg/bool.hpp"
#include "waypoint_manager_msgs/msg/waypoint.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"

namespace {
    static std::atomic_bool received_waypoint, stop_waypoint;
    static float default_goal_radius = 1;
    static float current_goal_radius = default_goal_radius;
    static Eigen::Vector2f current_position = Eigen::Vector2f::Zero();
    static Eigen::Vector2f old_current_position = Eigen::Vector2f::Zero();
    static time_t start_time, last_moving_time;
    static std::atomic_bool is_fst_flag, is_fst_waypoint_reached, is_reached_goal, is_to_prev_waypoint;
    static std::string old_id;
    static double delta_pose_dist = 0, pose_dist = 0;
    static float vel_x = 0;
    static float limit_delta_pose_dist = 0.1;
    static float limit_time = 20;
}

void waypointCallback(const waypoint_manager_msgs::msg::Waypoint::SharedPtr msg) {
    try {
        // first callback process
        if (is_fst_flag) {
            old_id = msg->identity;
            is_fst_flag.store(false);
            start_time = time(NULL);
            return;
        }

        // check switch waypoint
        if (msg->identity != old_id) {
            start_time = time(NULL);
            is_fst_waypoint_reached.store(true);
        }

        received_waypoint.store(true);
        old_id = msg->identity;
    }
    catch(const std::exception &) {
        received_waypoint.store(false);
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed parse check_robot_moving_node");
    }
}

void MclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    try {
        current_position.x() = msg->pose.pose.position.x;
        current_position.y() = msg->pose.pose.position.y;

        if(received_waypoint.load()) {
            delta_pose_dist = std::sqrt(std::pow(current_position.x() - old_current_position.x(), 2) + std::pow(current_position.y() - old_current_position.y(), 2)) * 5.0;
        }

        old_current_position = current_position;
    }
    catch(const std::exception &) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed mcl_pose");
    }
}

void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    try {
        vel_x = msg->linear.x;      
    }
    catch(const std::exception &) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed cmd_vel");
    }
}

void IsReachedGoalCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    try {
        is_reached_goal.store(msg->data);        
    }
    catch(const std::exception &) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed is_reached_goal");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("check_robot_moving_node");

    std::string goal_topic, waypoint_topic, is_reached_goal_topic, mcl_pose_topic, cmd_vel_topic, clear_costmap_srv;
    std::string robot_base_frame, global_frame;
    float goal_check_frequency, wait_no_waypoint_time;

    received_waypoint.store(false);
    stop_waypoint.store(false);
    is_fst_flag.store(true);
    is_fst_waypoint_reached.store(false);
    is_reached_goal.store(false);
    is_to_prev_waypoint.store(false);

    node->declare_parameter("goal_topic", "move_base_simple/goal");
    node->declare_parameter("waypoint", "waypoint");
    node->declare_parameter("is_reached_goal_topic", "waypoint/is_reached");
    node->declare_parameter("robot_base_frame", "base_link");
    node->declare_parameter("global_frame", "map");
    node->declare_parameter("goal_check_frequency", 1.0f);
    node->declare_parameter("wait_no_waypoint_time", 5.0f);
    node->declare_parameter("default_goal_radius", 1.0f);
    node->declare_parameter("mcl_pose_topic", "mcl_pose");
    node->declare_parameter("cmd_vel_topic", "icart_mini/cmd_vel");
    node->declare_parameter("clear_costmap_srv", "move_base/clear_costmaps");

    node->get_parameter("goal_topic", goal_topic);
    node->get_parameter("waypoint", waypoint_topic);
    node->get_parameter("is_reached_goal_topic", is_reached_goal_topic);
    node->get_parameter("robot_base_frame", robot_base_frame);
    node->get_parameter("global_frame", global_frame);
    node->get_parameter("goal_check_frequency", goal_check_frequency);
    node->get_parameter("wait_no_waypoint_time", wait_no_waypoint_time);
    node->get_parameter("default_goal_radius", default_goal_radius);
    node->get_parameter("mcl_pose_topic", mcl_pose_topic);
    node->get_parameter("cmd_vel_topic", cmd_vel_topic);
    node->get_parameter("clear_costmap_srv", clear_costmap_srv);

    auto waypoint_subscriber = node->create_subscription<waypoint_manager_msgs::msg::Waypoint>(
        waypoint_topic, 1, waypointCallback);

    auto mcl_pose_subscriber = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        mcl_pose_topic, 2, MclPoseCallback);

    auto cmd_vel_subscriber = node->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, 2, CmdVelCallback);

    auto is_reached_goal_subscriber = node->create_subscription<std_msgs::msg::Bool>(
        is_reached_goal_topic, 1, IsReachedGoalCallback);

    auto prev_waypoint_service = node->create_client<std_srvs::srv::Trigger>("waypoint_server/prev_waypoint");
    auto clear_costmap_service = node->create_client<std_srvs::srv::Empty>(clear_costmap_srv);

    RCLCPP_INFO(node->get_logger(), "Start check_robot_moving_node");

    rclcpp::Rate loop_rate(5);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        if (!received_waypoint.load()) {
            RCLCPP_INFO(node->get_logger(), "Waiting waypoint check_moving_node");
            rclcpp::sleep_for(std::chrono::seconds(1));
            continue;
        }

        // moving toward previous waypoint
        if (is_reached_goal && is_to_prev_waypoint) {
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            clear_costmap_service->async_send_request(request);
            is_to_prev_waypoint.store(false);
            RCLCPP_WARN(node->get_logger(), "Clear Costmaps");
        }

        // check robot delta pose dist
        if (delta_pose_dist <= limit_delta_pose_dist && !is_reached_goal) {
            if (time(NULL) - last_moving_time >= limit_time) {
                auto trigger_request = std::make_shared<std_srvs::srv::Trigger::Request>();
                if (is_fst_waypoint_reached) {
                    RCLCPP_INFO(node->get_logger(), "Service call PrevWaypoint()");
                    prev_waypoint_service->async_send_request(trigger_request);
                    is_to_prev_waypoint.store(true);
                    last_moving_time = time(NULL);
                }
            }
        } else {
            last_moving_time = time(NULL);
        }

        RCLCPP_INFO(node->get_logger(), "time:%ld, stopped time:%ld\n", time(NULL) - start_time, time(NULL) - last_moving_time);

        loop_rate.sleep();
    }
    RCLCPP_INFO(node->get_logger(), "Finish waypoint_server_node");

    rclcpp::shutdown();
    return 0;
}
