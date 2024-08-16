#include <limits>
#include <atomic>
#include <string>
#include <stdexcept>
#include <exception>
#include <cmath>
#include <iostream>
#include <algorithm>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <yaml-cpp/yaml.h>

#include "std_msgs/msg/bool.hpp"
#include "waypoint_manager_msgs/msg/waypoint.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

namespace {
    static std::atomic_bool received_waypoint, is_fst_flag, is_reconfigure;
    static float default_goal_radius = 1;
    static float current_goal_radius = default_goal_radius;
    static Eigen::Vector2f current_position = Eigen::Vector2f::Zero();
    static std::string old_id, file_path_, start_id, end_id, area_name;
    static float default_global_inflation, default_local_inflation, default_dwa_limit_vel;
    static YAML::Node yaml_config;
    static float global_inflation, local_inflation, dwa_limit_vel;
}

void change_global_inflation_param(const std::shared_ptr<rclcpp::Node>& node, const std::string& param_name, double value);
void change_local_inflation_param(const std::shared_ptr<rclcpp::Node>& node, const std::string& param_name, double value);
void change_dwa_param(const std::shared_ptr<rclcpp::Node>& node, const std::string& param_name, double value);

void waypointCallback(const waypoint_manager_msgs::msg::Waypoint::SharedPtr msg, const std::shared_ptr<rclcpp::Node>& node) {
    try {
        if (is_fst_flag) {
            old_id = msg->identity;
            is_fst_flag.store(false);
            return;
        }

        if (msg->identity != old_id) {
            for (const auto &wp : yaml_config["waypoint_reconfigure_config"]["areas"]) {
                area_name = wp["name"].as<std::string>();
                start_id = wp["start_id"].as<std::string>();
                end_id = wp["end_id"].as<std::string>();

                if (msg->identity == end_id && is_reconfigure) {
                    RCLCPP_WARN(node->get_logger(), "This %s area is now complete", area_name.c_str());

                    for (const auto &p : wp["properties"]) {
                        if (p["key"].as<std::string>() == "global_inflation") {
                            change_global_inflation_param(node, "inflation_radius", default_global_inflation);
                        }

                        if (p["key"].as<std::string>() == "local_inflation") {
                            change_local_inflation_param(node, "inflation_radius", default_local_inflation);
                        }

                        if (p["key"].as<std::string>() == "dwa_limit_vel") {
                            change_dwa_param(node, "max_vel_x", default_dwa_limit_vel);
                            change_dwa_param(node, "max_vel_trans", default_dwa_limit_vel);
                        }
                    }
                    is_reconfigure.store(false);
                }

                if (msg->identity == start_id && !is_reconfigure) {
                    is_reconfigure.store(true);
                    RCLCPP_WARN(node->get_logger(), "This area is %s", area_name.c_str());

                    for (const auto &p : wp["properties"]) {
                        if (p["key"].as<std::string>() == "global_inflation") {
                            global_inflation = p["value"].as<float>();
                            RCLCPP_WARN(node->get_logger(), "Set global_inflation %f", global_inflation);
                            change_global_inflation_param(node, "inflation_radius", global_inflation);
                        }

                        if (p["key"].as<std::string>() == "local_inflation") {
                            local_inflation = p["value"].as<float>();
                            RCLCPP_WARN(node->get_logger(), "Set local_inflation %f", local_inflation);
                            change_local_inflation_param(node, "inflation_radius", local_inflation);
                        }

                        if (p["key"].as<std::string>() == "dwa_limit_vel") {
                            dwa_limit_vel = p["value"].as<float>();
                            RCLCPP_WARN(node->get_logger(), "Set dwa_limit_vel %f", dwa_limit_vel);
                            change_dwa_param(node, "max_vel_x", dwa_limit_vel);
                            change_dwa_param(node, "max_vel_trans", dwa_limit_vel);
                        }
                    }
                }
            }
        }

        old_id = msg->identity;
        received_waypoint.store(true);
    }
    catch(const std::exception &) {
        received_waypoint.store(false);
        RCLCPP_WARN(node->get_logger(), "Failed to parse waypoint_reconfigure_node");
    }
}

void readYaml(const std::shared_ptr<rclcpp::Node>& node) {
    try {
        file_path_ = node->declare_parameter<std::string>("file_path", node->get_name() + "/config/reconfigure_list.yaml");
        RCLCPP_INFO(node->get_logger(), "Load %s", file_path_.c_str());
        yaml_config = YAML::LoadFile(file_path_);

        default_global_inflation = yaml_config["waypoint_reconfigure_config"]["default_global_inflation"].as<float>();
        default_local_inflation = yaml_config["waypoint_reconfigure_config"]["default_local_inflation"].as<float>();
        default_dwa_limit_vel = yaml_config["waypoint_reconfigure_config"]["default_dwa_limit_vel"].as<float>();
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error reading YAML: %s", e.what());
    }
}

void change_global_inflation_param(const std::shared_ptr<rclcpp::Node>& node, const std::string& param_name, double value) {
    auto client = node->create_client<rcl_interfaces::srv::SetParameters>("/move_base/global_costmap/inflation_layer/set_parameters");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node->get_logger(), "Waiting for global inflation service...");
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rclcpp::Parameter param(param_name, value);
    request->parameters.push_back(param.to_parameter_msg());

    auto future = client->async_send_request(request);
    try {
        auto response = future.get();
        RCLCPP_INFO(node->get_logger(), "Global inflation parameter set to %f", value);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call global inflation service: %s", e.what());
    }
}

void change_local_inflation_param(const std::shared_ptr<rclcpp::Node>& node, const std::string& param_name, double value) {
    auto client = node->create_client<rcl_interfaces::srv::SetParameters>("/move_base/local_costmap/inflation_layer/set_parameters");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node->get_logger(), "Waiting for local inflation service...");
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rclcpp::Parameter param(param_name, value);
    request->parameters.push_back(param.to_parameter_msg());

    auto future = client->async_send_request(request);
    try {
        auto response = future.get();
        RCLCPP_INFO(node->get_logger(), "Local inflation parameter set to %f", value);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call local inflation service: %s", e.what());
    }
}

void change_dwa_param(const std::shared_ptr<rclcpp::Node>& node, const std::string& param_name, double value) {
    auto client = node->create_client<rcl_interfaces::srv::SetParameters>("/move_base/DWAPlannerROS/set_parameters");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node->get_logger(), "Waiting for DWAPlanner service...");
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rclcpp::Parameter param(param_name, value);
    request->parameters.push_back(param.to_parameter_msg());

    auto future = client->async_send_request(request);
    try {
        auto response = future.get();
        RCLCPP_INFO(node->get_logger(), "DWAPlanner parameter set to %f", value);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call DWAPlanner service: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("waypoint_reconfigure_node");

    std::string goal_topic, waypoint_topic, is_reached_goal_topic;
    std::string robot_base_frame, global_frame;

    readYaml(node);

    received_waypoint.store(false);
    is_fst_flag.store(true);
    is_reconfigure.store(false);

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

    auto waypoint_subscriber = node->create_subscription<waypoint_manager_msgs::msg::Waypoint>(
        waypoint_topic, 1, std::bind(waypointCallback, std::placeholders::_1, node)
    );

    RCLCPP_INFO(node->get_logger(), "Start waypoint_reconfigure_node");

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Finish waypoint_reconfigure_node");

    rclcpp::shutdown();
    return 0;
}
