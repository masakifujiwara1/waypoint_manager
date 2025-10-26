
#include <limits>
#include <atomic>
#include <string>
#include <stdexcept>
#include <exception>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <memory>
#include <chrono>
#include <filesystem>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <std_msgs/msg/bool.hpp>
#include <waypoint_manager_msgs/msg/waypoint.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>

class WaypointReconfigureNode : public rclcpp::Node {
public:
    WaypointReconfigureNode() : Node("waypoint_reconfigure_node") {
        // Declare parameters
        this->declare_parameter("file_path", "");
        this->declare_parameter("goal_topic", "move_base_simple/goal");
        this->declare_parameter("waypoint", "waypoint");
        this->declare_parameter("is_reached_goal_topic", "waypoint/is_reached");
        this->declare_parameter("robot_base_frame", "base_link");
        this->declare_parameter("global_frame", "map");
        this->declare_parameter("goal_check_frequency", 1.0);
        this->declare_parameter("wait_no_waypoint_time", 5.0);
        this->declare_parameter("default_goal_radius", 1.0);

        // Initialize state
        recived_waypoint_.store(false);
        is_fst_flag_.store(true);
        is_reconfigure_.store(false);

        // Read YAML configuration
        readYaml();

        // Get parameters
        std::string waypoint_topic = this->get_parameter("waypoint").as_string();

        // Create subscriber
        waypoint_subscriber_ = this->create_subscription<waypoint_manager_msgs::msg::Waypoint>(
            waypoint_topic,
            10,
            std::bind(&WaypointReconfigureNode::waypointCallback, this, std::placeholders::_1)
        );

        // Create service clients for dynamic reconfigure
        global_inflation_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
            "/move_base/global_costmap/inflation_layer/set_parameters");
        local_inflation_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
            "/move_base/local_costmap/inflation_layer/set_parameters");
        local_cost_cloud_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
            "/move_base/local_costmap/local_cost_cloud_layer/set_parameters");
        dwa_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
            "/move_base/DWAPlannerROS/set_parameters");

        RCLCPP_INFO(this->get_logger(), "Start waypoint_reconfigure_node");
    }

private:
    std::atomic_bool recived_waypoint_;
    std::atomic_bool is_fst_flag_;
    std::atomic_bool is_reconfigure_;
    
    double default_goal_radius_;
    double current_goal_radius_;
    std::string old_id_;
    std::string file_path_;
    std::string start_id_;
    std::string end_id_;
    std::string area_name_;
    double default_global_inflation_;
    double default_local_inflation_;
    double default_dwa_limit_vel_;
    double global_inflation_;
    double local_inflation_;
    double dwa_limit_vel_;
    
    YAML::Node yaml_config_;

    rclcpp::Subscription<waypoint_manager_msgs::msg::Waypoint>::SharedPtr waypoint_subscriber_;
    
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr global_inflation_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr local_inflation_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr local_cost_cloud_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr dwa_client_;

    void readYaml() {
        try {
            file_path_ = this->get_parameter("file_path").as_string();
            if (file_path_.empty()) {
                std::string package_path = ament_index_cpp::get_package_share_directory("waypoint_reconfigure");
                file_path_ = package_path + "/config/reconfigure_list.yaml";
            }
            
            RCLCPP_INFO(this->get_logger(), "Load %s", file_path_.c_str());
            yaml_config_ = YAML::LoadFile(file_path_);

            default_global_inflation_ = yaml_config_["waypoint_reconfigure_config"]["default_global_inflation"].as<double>();
            default_local_inflation_ = yaml_config_["waypoint_reconfigure_config"]["default_local_inflation"].as<double>();
            default_dwa_limit_vel_ = yaml_config_["waypoint_reconfigure_config"]["default_dwa_limit_vel"].as<double>();
        }
        catch(const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

    void changeParameter(
        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client,
        const std::string& param_name, 
        const rclcpp::ParameterValue& value)
    {
        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return;
        }

        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter param;
        param.name = param_name;
        param.value = value.to_value_msg();
        request->parameters.push_back(param);

        client->async_send_request(request);
    }

    void changeGlobalInflationParam(const std::string& param_name, double value) {
        changeParameter(global_inflation_client_, param_name, rclcpp::ParameterValue(value));
    }

    void changeLocalInflationParam(const std::string& param_name, double value) {
        changeParameter(local_inflation_client_, param_name, rclcpp::ParameterValue(value));
    }

    void changeLocalCostCloudParam(const std::string& param_name, bool value) {
        changeParameter(local_cost_cloud_client_, param_name, rclcpp::ParameterValue(value));
    }

    void changeDwaParam(const std::string& param_name, double value) {
        changeParameter(dwa_client_, param_name, rclcpp::ParameterValue(value));
    }

    void waypointCallback(const waypoint_manager_msgs::msg::Waypoint::SharedPtr msg) {
        try {
            if (is_fst_flag_.load()) {
                old_id_ = msg->identity;
                is_fst_flag_.store(false);
                return;
            }

            // check switch waypoint
            if (msg->identity != old_id_) {
                // 現在のwpと合致するエリアのスタートwpを全探索
                for (const auto &wp : yaml_config_["waypoint_reconfigure_config"]["areas"]) {
                    area_name_ = wp["name"].as<std::string>();
                    start_id_ = wp["start_id"].as<std::string>();
                    end_id_ = wp["end_id"].as<std::string>();

                    // エリア終了判定
                    if (msg->identity == end_id_ && is_reconfigure_.load()) {
                        RCLCPP_WARN(this->get_logger(), "This %s area is now complete", area_name_.c_str());

                        // reconfig with default param
                        for (const auto &p : wp["properties"]) {
                            if (p["key"].as<std::string>() == "global_inflation") {
                                changeGlobalInflationParam("inflation_radius", default_global_inflation_);
                            }

                            if (p["key"].as<std::string>() == "local_inflation") {
                                changeLocalInflationParam("inflation_radius", default_local_inflation_);
                            }

                            if (p["key"].as<std::string>() == "local_cost_cloud") {
                                changeLocalCostCloudParam("enabled", true);
                            }

                            if (p["key"].as<std::string>() == "dwa_limit_vel") {
                                changeDwaParam("max_vel_x", default_dwa_limit_vel_);
                                changeDwaParam("max_vel_trans", default_dwa_limit_vel_);
                            }
                        }
                        is_reconfigure_.store(false);
                    }

                    // エリア開始判定
                    if (msg->identity == start_id_ && !is_reconfigure_.load()) {
                        is_reconfigure_.store(true);
                        RCLCPP_WARN(this->get_logger(), "This area is %s", area_name_.c_str());

                        // reconfig
                        for (const auto &p : wp["properties"]) {
                            if (p["key"].as<std::string>() == "global_inflation") {
                                global_inflation_ = p["value"].as<double>();
                                RCLCPP_WARN(this->get_logger(), "Set global_inflation %f", global_inflation_);
                                changeGlobalInflationParam("inflation_radius", global_inflation_);
                            }

                            if (p["key"].as<std::string>() == "local_inflation") {
                                local_inflation_ = p["value"].as<double>();
                                RCLCPP_WARN(this->get_logger(), "Set local_inflation %f", local_inflation_);
                                changeLocalInflationParam("inflation_radius", local_inflation_);
                            }

                            if (p["key"].as<std::string>() == "local_cost_cloud") {
                                changeLocalCostCloudParam("enabled", false);
                            }

                            if (p["key"].as<std::string>() == "dwa_limit_vel") {
                                dwa_limit_vel_ = p["value"].as<double>();
                                RCLCPP_WARN(this->get_logger(), "Set dwa_limit_vel %f", dwa_limit_vel_);
                                changeDwaParam("max_vel_x", dwa_limit_vel_);
                                changeDwaParam("max_vel_trans", dwa_limit_vel_);
                            }
                        }
                    }
                }
            }

            old_id_ = msg->identity;
            recived_waypoint_.store(true);
        }
        catch(const std::exception &e) {
            recived_waypoint_.store(false);
            RCLCPP_WARN(this->get_logger(), "Failed parse waypoint_reconfigure_node: %s", e.what());
        }
    }
};

auto main(int argc, char **argv) -> int {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WaypointReconfigureNode>();
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "Finish waypoint_reconfigure_node");
    rclcpp::shutdown();

    return 0;
}

