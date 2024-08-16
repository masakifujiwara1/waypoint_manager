#include <limits>
#include <atomic>
#include <string>
#include <stdexcept>
#include <exception>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "std_msgs/msg/bool.hpp"
#include "waypoint_manager_msgs/msg/waypoint.hpp"

namespace {
    static std::atomic_bool received_waypoint;
    static float default_goal_radius = 1.0;
    static float current_goal_radius = default_goal_radius;
    static Eigen::Vector2f goal_position = Eigen::Vector2f::Zero();
}

void waypointCallback(const waypoint_manager_msgs::msg::Waypoint::SharedPtr msg) {
    try {
        goal_position.x() = msg->pose.position.x;
        goal_position.y() = msg->pose.position.y;

        bool found_goal_radius = false;

        for (const auto& property : msg->properties) {
            if (property.name == "goal_radius") {
                current_goal_radius = std::stof(property.data);
                found_goal_radius = true;
            }
        }

        if (!found_goal_radius) {
            current_goal_radius = default_goal_radius;
        }
        received_waypoint.store(true);
    }
    catch (const std::exception &) {
        received_waypoint.store(false);
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to parse radius_node");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("radius_node");

    std::string goal_topic, waypoint_topic, is_reached_goal_topic;
    std::string robot_base_frame, global_frame;

    float goal_check_frequency, wait_no_waypoint_time;

    received_waypoint.store(false);

    node->declare_parameter("goal_topic", "move_base_simple/goal");
    node->declare_parameter("waypoint", "waypoint");
    node->declare_parameter("is_reached_goal_topic", "waypoint/is_reached");
    node->declare_parameter("robot_base_frame", "base_link");
    node->declare_parameter("global_frame", "map");
    node->declare_parameter("goal_check_frequency", 1.0f);
    node->declare_parameter("wait_no_waypoint_time", 5.0f);
    node->declare_parameter("default_goal_radius", 1.0f);

    node->get_parameter("goal_topic", goal_topic);
    node->get_parameter("waypoint", waypoint_topic);
    node->get_parameter("is_reached_goal_topic", is_reached_goal_topic);
    node->get_parameter("robot_base_frame", robot_base_frame);
    node->get_parameter("global_frame", global_frame);
    node->get_parameter("goal_check_frequency", goal_check_frequency);
    node->get_parameter("wait_no_waypoint_time", wait_no_waypoint_time);
    node->get_parameter("default_goal_radius", default_goal_radius);

    current_goal_radius = default_goal_radius;

    if (robot_base_frame == global_frame) {
        throw std::runtime_error("Please set different frame names for robot_base_frame and global_frame");
    }

    auto loop_rate = rclcpp::Rate(goal_check_frequency);
    auto is_reached_goal_publisher = node->create_publisher<std_msgs::msg::Bool>(is_reached_goal_topic, 1);
    auto waypoint_subscriber = node->create_subscription<waypoint_manager_msgs::msg::Waypoint>(
        waypoint_topic, 1, waypointCallback
    );

    RCLCPP_INFO(node->get_logger(), "Start radius_node");
    RCLCPP_INFO(node->get_logger(), "robot_base_frame is %s", robot_base_frame.c_str());
    RCLCPP_INFO(node->get_logger(), "global_frame is %s", global_frame.c_str());
    RCLCPP_INFO(node->get_logger(), "default_goal_radius is %f", default_goal_radius);

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    while (rclcpp::ok()) {
        Eigen::Vector2f distance_of_goal;

        rclcpp::spin_some(node);

        if (!received_waypoint.load()) {
            RCLCPP_INFO(node->get_logger(), "Waiting for waypoint in radius_node");
            rclcpp::sleep_for(std::chrono::seconds(static_cast<int>(wait_no_waypoint_time)));
            continue;
        }

        try {
            geometry_msgs::msg::TransformStamped tf_transform;
            tf_transform = tf_buffer.lookupTransform(global_frame, robot_base_frame, tf2::TimePointZero);

            distance_of_goal.x() = goal_position.x() - tf_transform.transform.translation.x;
            distance_of_goal.y() = goal_position.y() - tf_transform.transform.translation.y;

            std_msgs::msg::Bool msg;

            if (distance_of_goal.norm() < current_goal_radius) {
                RCLCPP_INFO(node->get_logger(), "Goal reached from goal_event_handler::radius_node");
                msg.data = true;
            } else {
                msg.data = false;
            }

            is_reached_goal_publisher->publish(msg);
        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(node->get_logger(), "Transform failed from %s to %s: %s", global_frame.c_str(), robot_base_frame.c_str(), ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        loop_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Finish radius_node");

    rclcpp::shutdown();
    return 0;
}
