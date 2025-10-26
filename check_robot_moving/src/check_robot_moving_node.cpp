
#include <limits>
#include <atomic>
#include <string>
#include <stdexcept>
#include <exception>
#include <cmath>
#include <iostream>
#include <memory>
#include <chrono>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <waypoint_manager_msgs/msg/waypoint.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class CheckRobotMovingNode : public rclcpp::Node {
public:
    CheckRobotMovingNode() : Node("check_robot_moving_node") {
        // Declare parameters
        this->declare_parameter("goal_topic", "move_base_simple/goal");
        this->declare_parameter("waypoint", "waypoint");
        this->declare_parameter("is_reached_goal_topic", "waypoint/is_reached");
        this->declare_parameter("robot_base_frame", "base_link");
        this->declare_parameter("global_frame", "map");
        this->declare_parameter("goal_check_frequency", 1.0);
        this->declare_parameter("wait_no_waypoint_time", 5.0);
        this->declare_parameter("default_goal_radius", 1.0);
        this->declare_parameter("mcl_pose_topic", "mcl_pose");
        this->declare_parameter("cmd_vel_topic", "icart_mini/cmd_vel");
        this->declare_parameter("clear_costmap_srv", "move_base/clear_costmaps");
        this->declare_parameter("limit_time", 20.0);
        this->declare_parameter("limit_delta_pose_dist", 0.1);

        // Get parameters
        std::string goal_topic = this->get_parameter("goal_topic").as_string();
        std::string waypoint_topic = this->get_parameter("waypoint").as_string();
        std::string is_reached_goal_topic = this->get_parameter("is_reached_goal_topic").as_string();
        robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
        global_frame_ = this->get_parameter("global_frame").as_string();
        goal_check_frequency_ = this->get_parameter("goal_check_frequency").as_double();
        wait_no_waypoint_time_ = this->get_parameter("wait_no_waypoint_time").as_double();
        default_goal_radius_ = this->get_parameter("default_goal_radius").as_double();
        std::string mcl_pose_topic = this->get_parameter("mcl_pose_topic").as_string();
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        std::string clear_costmap_srv = this->get_parameter("clear_costmap_srv").as_string();
        limit_time_ = this->get_parameter("limit_time").as_double();
        limit_delta_pose_dist_ = this->get_parameter("limit_delta_pose_dist").as_double();

        // Initialize state
        recived_waypoint_.store(false);
        stop_waypoint_.store(false);
        is_fst_flag_.store(true);
        is_fst_waypoint_reached_.store(false);
        is_reached_goal_.store(false);
        is_to_prev_waypoint_.store(false);

        current_goal_radius_ = default_goal_radius_;
        current_position_ = Eigen::Vector2f::Zero();
        old_current_position_ = Eigen::Vector2f::Zero();
        delta_pose_dist_ = 0;
        pose_dist_ = 0;
        vel_x_ = 0;

        // Create subscribers
        waypoint_subscriber_ = this->create_subscription<waypoint_manager_msgs::msg::Waypoint>(
            waypoint_topic,
            10,
            std::bind(&CheckRobotMovingNode::waypointCallback, this, std::placeholders::_1)
        );
        mcl_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            mcl_pose_topic,
            10,
            std::bind(&CheckRobotMovingNode::mclPoseCallback, this, std::placeholders::_1)
        );
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic,
            10,
            std::bind(&CheckRobotMovingNode::cmdVelCallback, this, std::placeholders::_1)
        );
        is_reached_goal_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            is_reached_goal_topic,
            10,
            std::bind(&CheckRobotMovingNode::isReachedGoalCallback, this, std::placeholders::_1)
        );

        // Create service clients
        prev_waypoint_client_ = this->create_client<std_srvs::srv::Trigger>("waypoint_server/prev_waypoint");
        clear_costmap_client_ = this->create_client<std_srvs::srv::Empty>(clear_costmap_srv);

        RCLCPP_INFO(this->get_logger(), "Start check_robot_moving_node");

        // Initialize time
        start_time_ = this->now();
        last_moving_time_ = this->now();

        // Create timer for main loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),  // 5 Hz
            std::bind(&CheckRobotMovingNode::checkLoop, this)
        );
    }

private:
    std::atomic_bool recived_waypoint_, stop_waypoint_;
    std::atomic_bool is_fst_flag_, is_fst_waypoint_reached_, is_reached_goal_, is_to_prev_waypoint_;
    
    double default_goal_radius_;
    double current_goal_radius_;
    Eigen::Vector2f current_position_;
    Eigen::Vector2f old_current_position_;
    std::string old_id_;
    double delta_pose_dist_;
    double pose_dist_;
    float vel_x_;
    double limit_delta_pose_dist_;
    double limit_time_;

    std::string robot_base_frame_;
    std::string global_frame_;
    double goal_check_frequency_;
    double wait_no_waypoint_time_;

    rclcpp::Time start_time_;
    rclcpp::Time last_moving_time_;

    rclcpp::Subscription<waypoint_manager_msgs::msg::Waypoint>::SharedPtr waypoint_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr mcl_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_reached_goal_subscriber_;
    
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr prev_waypoint_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmap_client_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    void waypointCallback(const waypoint_manager_msgs::msg::Waypoint::SharedPtr msg) {
        try {
            // first callback process
            if (is_fst_flag_.load()) {
                old_id_ = msg->identity;
                is_fst_flag_.store(false);
                start_time_ = this->now();
                return;
            }

            // check switch waypoint
            if (msg->identity != old_id_) {
                start_time_ = this->now();
                is_fst_waypoint_reached_.store(true);
            }

            recived_waypoint_.store(true);
            old_id_ = msg->identity;
        }
        catch(const std::exception &) {
            recived_waypoint_.store(false);
            RCLCPP_WARN(this->get_logger(), "Failed parse check_robot_moving_node");
        }
    }

    void mclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        try {
            current_position_.x() = msg->pose.pose.position.x;
            current_position_.y() = msg->pose.pose.position.y;

            if(recived_waypoint_.load()) {
                delta_pose_dist_ = std::sqrt(
                    std::pow(current_position_.x() - old_current_position_.x(), 2) + 
                    std::pow(current_position_.y() - old_current_position_.y(), 2)
                ) * 5.0;
            }

            old_current_position_ = current_position_;
        }
        catch(const std::exception &) {
            RCLCPP_WARN(this->get_logger(), "Failed mcl_pose");
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        try {
            vel_x_ = msg->linear.x;
        }
        catch(const std::exception &) {
            RCLCPP_WARN(this->get_logger(), "Failed cmd_vel");
        }
    }

    void isReachedGoalCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        try {
            is_reached_goal_.store(msg->data);
        }
        catch(const std::exception &) {
            RCLCPP_WARN(this->get_logger(), "Failed is_reached_goal");
        }
    }

    void checkLoop() {
        if (!recived_waypoint_.load()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Waiting waypoint check_moving_node");
            last_moving_time_ = this->now();
            return;
        }

        // moving toward previous waypoint
        if (is_reached_goal_.load() && is_to_prev_waypoint_.load()) {
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            if (clear_costmap_client_->wait_for_service(std::chrono::seconds(0))) {
                clear_costmap_client_->async_send_request(request);
                is_to_prev_waypoint_.store(false);
                RCLCPP_WARN(this->get_logger(), "Clear Costmaps");
            }
        }

        // check robot delta pose dist
        if (delta_pose_dist_ <= limit_delta_pose_dist_ && !is_reached_goal_.load()) {
            auto current_time = this->now();
            auto stopped_duration = (current_time - last_moving_time_).seconds();
            auto total_duration = (current_time - start_time_).seconds();

            RCLCPP_INFO(this->get_logger(), "time:%.1f, stopped time:%.1f", 
                       total_duration, stopped_duration);

            if (stopped_duration >= limit_time_) {
                if (is_fst_waypoint_reached_.load()) {
                    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                    if (prev_waypoint_client_->wait_for_service(std::chrono::seconds(0))) {
                        RCLCPP_INFO(this->get_logger(), "Service call PrevWaypoint()");
                        prev_waypoint_client_->async_send_request(request);
                        is_to_prev_waypoint_.store(true);
                        last_moving_time_ = this->now();
                    }
                }
            }
        } else {
            last_moving_time_ = this->now();
        }
    }
};

auto main(int argc, char **argv) -> int {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CheckRobotMovingNode>();
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "Finish check_robot_moving_node");
    rclcpp::shutdown();

    return 0;
}

