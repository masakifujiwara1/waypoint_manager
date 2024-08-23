
#include <limits>
#include <atomic>
#include <string>
#include <stdexcept>
#include <exception>
#include <cmath>
#include <iostream>
#include <algorithm>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_listener.h>
#include <time.h>
#include <yaml-cpp/yaml.h>

#include <std_msgs/Bool.h>
#include <waypoint_manager_msgs/Waypoint.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/client.h>
#include "costmap_2d/InflationPluginConfig.h"
#include "costmap_2d/VoxelPluginConfig.h"
#include "dwa_local_planner/DWAPlannerConfig.h"

namespace {
    static std::atomic_bool recived_waypoint, is_fst_flag, is_reconfigure;
    static float default_goal_radius = 1;
    static float current_goal_radius = default_goal_radius;
    static Eigen::Vector2f current_position = Eigen::Vector2f::Zero();
    static std::string old_id, file_path_, start_id, end_id, area_name;
    static float default_global_inflation, default_local_inflation, default_dwa_limit_vel;
    static YAML::Node yaml_config;
    static float global_inflation, local_inflation, dwa_limit_vel;
}

void change_global_inflation_param(const std::string& param_name, double value);
void change_local_inflation_param(const std::string& param_name, double value);
void change_local_cost_cloud_param(const std::string& param_name, bool value);
void change_dwa_param(const std::string& param_name, double value);

void waypointCallback(const waypoint_manager_msgs::Waypoint::ConstPtr &msg) {
    try {
        // ROS_WARN("recived_waypoint");        
        if (is_fst_flag) {
            old_id = msg->identity;
            is_fst_flag.store(false);
            return;
        }

        // check switch waypoint
        if (msg->identity != old_id) {
            // 現在のwpと合致するエリアのスタートwpを全探索
            for (const auto &wp : yaml_config["waypoint_reconfigure_config"]["areas"]) {
                area_name = wp["name"].as<std::string>();
                start_id = wp["start_id"].as<std::string>();
                end_id = wp["end_id"].as<std::string>();

                // エリア終了判定
                if (msg->identity == end_id && is_reconfigure) {
                    ROS_WARN("This %s area is now complete", area_name.c_str());

                    // reconfig with default param
                    for (const auto &p : wp["properties"]) {
                        if (p["key"].as<std::string>() == "global_inflation") {
                            change_global_inflation_param("inflation_radius", default_global_inflation);
                        }

                        if (p["key"].as<std::string>() == "local_inflation") {
                            change_local_inflation_param("inflation_radius", default_local_inflation);
                        }

                        if (p["key"].as<std::string>() == "local_cost_cloud") {
                            change_local_cost_cloud_param("enabled", true);
                        }

                        if (p["key"].as<std::string>() == "dwa_limit_vel") {
                            change_dwa_param("max_vel_x", default_dwa_limit_vel);
                            change_dwa_param("max_vel_trans", default_dwa_limit_vel);
                        }
                    }
                    is_reconfigure.store(false);
                }

                // エリア開始判定
                if (msg->identity == start_id && !is_reconfigure) {
                    is_reconfigure.store(true);
                    // ROS_WARN("Switch %s", start_id.c_str());
                    ROS_WARN("This area is %s", area_name.c_str());

                    // reconfig
                    for (const auto &p : wp["properties"]) {
                        if (p["key"].as<std::string>() == "global_inflation") {
                            global_inflation = p["value"].as<float>();
                            ROS_WARN("Set global_inflation %f", global_inflation);
                            change_global_inflation_param("inflation_radius", global_inflation);
                        }

                        if (p["key"].as<std::string>() == "local_inflation") {
                            local_inflation = p["value"].as<float>();
                            ROS_WARN("Set local_inflation %f", local_inflation);
                            change_local_inflation_param("inflation_radius", local_inflation);
                        }

                        if (p["key"].as<std::string>() == "local_cost_cloud") {
                            // local_cost_cloud = p["value"].as<bool>();
                            // ROS_WARN("Set local_cost_cloud %f", local_inflation);
                            change_local_cost_cloud_param("enabled", false);
                        }

                        if (p["key"].as<std::string>() == "dwa_limit_vel") {
                            dwa_limit_vel = p["value"].as<float>();
                            ROS_WARN("Set dwa_limit_vel %f", dwa_limit_vel);
                            change_dwa_param("max_vel_x", dwa_limit_vel);
                            change_dwa_param("max_vel_trans", dwa_limit_vel);
                        }
                    }
                }
            }
        }

        // ROS_WARN("%s", msg->identity.c_str());        

        old_id = msg->identity;

        recived_waypoint.store(true);
    }
    catch(const std::exception &) {
        recived_waypoint.store(false);
        ROS_WARN("Failed parse waypoint_reconfigure_node");
    }
}

void readYaml(ros::NodeHandle& private_nh) {
    try{
        private_nh.param("file_path", file_path_, std::string(ros::package::getPath("waypoint_reconfigure") += "/config/reconfigure_list.yaml"));
        ROS_INFO("Load %s", file_path_.c_str());
        yaml_config = YAML::LoadFile(file_path_);

        default_global_inflation = yaml_config["waypoint_reconfigure_config"]["default_global_inflation"].as<float>();
        default_local_inflation = yaml_config["waypoint_reconfigure_config"]["default_local_inflation"].as<float>();
        default_dwa_limit_vel = yaml_config["waypoint_reconfigure_config"]["default_dwa_limit_vel"].as<float>();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
    }
}

void change_global_inflation_param(const std::string& param_name, double value) {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config config;

    double_param.name = param_name;
    double_param.value = value;
    config.doubles.push_back(double_param);

    srv_req.config = config;

    ros::service::call("/move_base/global_costmap/inflation_layer/set_parameters", srv_req, srv_resp);
}

void change_local_inflation_param(const std::string& param_name, double value) {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config config;

    double_param.name = param_name;
    double_param.value = value;
    config.doubles.push_back(double_param);

    srv_req.config = config;

    ros::service::call("/move_base/local_costmap/inflation_layer/set_parameters", srv_req, srv_resp);
}

void change_local_cost_cloud_param(const std::string& param_name, bool value) {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::Config config;

    bool_param.name = param_name;
    bool_param.value = value;
    config.bools.push_back(bool_param);

    srv_req.config = config;

    ros::service::call("/move_base/local_costmap/local_cost_cloud_layer/set_parameters", srv_req, srv_resp);
}

void change_dwa_param(const std::string& param_name, double value) {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config config;

    double_param.name = param_name;
    double_param.value = value;
    config.doubles.push_back(double_param);

    srv_req.config = config;

    ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

auto main(int argc, char **argv) -> int {
    ros::init(argc, argv, "waypoint_reconfigure_node");
    ros::NodeHandle nh,
                    private_nh("~");

    std::string goal_topic,
                waypoint_topic,
                is_reached_goal_topic;

    std::string robot_base_frame,
                global_frame;

    float goal_check_frequency,
          wait_no_waypoint_time;

    readYaml(private_nh);

    recived_waypoint.store(false);
    is_fst_flag.store(true);
    is_reconfigure.store(false);

    private_nh.param(
        "goal_topic",
        goal_topic,
        std::string("move_base_simple/goal")
    );
    private_nh.param(
        "waypoint",
        waypoint_topic,
        std::string("waypoint")
    );
    private_nh.param(
        "is_reached_goal_topic",
        is_reached_goal_topic,
        std::string("waypoint/is_reached")
    );
    private_nh.param(
        "robot_base_frame",
        robot_base_frame,
        std::string("base_link")
    );
    private_nh.param(
        "global_frame",
        global_frame,
        std::string("map")
    );
    private_nh.param(
        "goal_check_frequency",
        goal_check_frequency,
        static_cast<float>(1)
    );
    private_nh.param(
        "wait_no_waypoint_time",
        wait_no_waypoint_time,
        static_cast<float>(5.0)
    );
    private_nh.param(
        "default_goal_radius",
        default_goal_radius,
        static_cast<float>(1.0)
    );

    auto loop_rate = ros::Rate(5);
    auto waypoint_subscriber = nh.subscribe(
        waypoint_topic,
        1,
        waypointCallback
    );

    ROS_INFO("Start waypoint_reconfigure_node");

    ros::spin();

    ROS_INFO("Finish waypoint_server_node");

    return 0;
}

