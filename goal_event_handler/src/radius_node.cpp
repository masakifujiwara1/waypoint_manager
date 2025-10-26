
#include <limits>
#include <atomic>
#include <string>
#include <stdexcept>
#include <exception>
#include <memory>
#include <chrono>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_msgs/msg/bool.hpp>
#include <waypoint_manager_msgs/msg/waypoint.hpp>

class RadiusNode : public rclcpp::Node {
public:
    RadiusNode() : Node("radius_node") {
        // Declare parameters
        this->declare_parameter("goal_topic", "move_base_simple/goal");
        this->declare_parameter("waypoint", "waypoint");
        this->declare_parameter("is_reached_goal_topic", "waypoint/is_reached");
        this->declare_parameter("robot_base_frame", "base_link");
        this->declare_parameter("global_frame", "map");
        this->declare_parameter("goal_check_frequency", 1.0);
        this->declare_parameter("wait_no_waypoint_time", 5.0);
        this->declare_parameter("default_goal_radius", 1.0);

        // Get parameters
        std::string goal_topic = this->get_parameter("goal_topic").as_string();
        waypoint_topic_ = this->get_parameter("waypoint").as_string();
        is_reached_goal_topic_ = this->get_parameter("is_reached_goal_topic").as_string();
        robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
        global_frame_ = this->get_parameter("global_frame").as_string();
        goal_check_frequency_ = this->get_parameter("goal_check_frequency").as_double();
        wait_no_waypoint_time_ = this->get_parameter("wait_no_waypoint_time").as_double();
        default_goal_radius_ = this->get_parameter("default_goal_radius").as_double();

        current_goal_radius_ = default_goal_radius_;
        recived_waypoint_.store(false);

        if(robot_base_frame_ == global_frame_) {
            throw std::runtime_error("Please set different frame name robot_base_frame or global_frame");
        }

        // QoS for latched publisher
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

        is_reached_goal_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            is_reached_goal_topic_,
            qos
        );
        waypoint_subscriber_ = this->create_subscription<waypoint_manager_msgs::msg::Waypoint>(
            waypoint_topic_,
            10,
            std::bind(&RadiusNode::waypointCallback, this, std::placeholders::_1)
        );

        // TF2 setup
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Start radius_node");
        RCLCPP_INFO(this->get_logger(), "robot_base_frame is %s", robot_base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "global_base_frame is %s", global_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "default_goal_radius is %f", default_goal_radius_);

        // Create timer for goal checking
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / goal_check_frequency_),
            std::bind(&RadiusNode::checkGoalReached, this)
        );
    }

private:
    std::atomic_bool recived_waypoint_;
    double default_goal_radius_;
    double current_goal_radius_;
    Eigen::Vector2f goal_position_ = Eigen::Vector2f::Zero();

    std::string waypoint_topic_;
    std::string is_reached_goal_topic_;
    std::string robot_base_frame_;
    std::string global_frame_;
    double goal_check_frequency_;
    double wait_no_waypoint_time_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_reached_goal_publisher_;
    rclcpp::Subscription<waypoint_manager_msgs::msg::Waypoint>::SharedPtr waypoint_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void waypointCallback(const waypoint_manager_msgs::msg::Waypoint::SharedPtr msg) {
        try {
            goal_position_.x() = msg->pose.position.x;
            goal_position_.y() = msg->pose.position.y;

            bool found_goal_radius = false;

            for(size_t i = 0; i < msg->properties.size(); i++) {
                if(msg->properties[i].name == "goal_radius") {
                    current_goal_radius_ = std::stof(msg->properties[i].data);
                    found_goal_radius = true;
                }
            }
            if(!found_goal_radius) {
                current_goal_radius_ = default_goal_radius_;
            }
            recived_waypoint_.store(true);
        }
        catch(const std::exception &) {
            recived_waypoint_.store(false);
            RCLCPP_WARN(this->get_logger(), "Failed parse radius_node");
        }
    }

    void checkGoalReached() {
        if(!recived_waypoint_.load()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting waypoint radius_node");
            return;
        }

        try {
            geometry_msgs::msg::TransformStamped tf_transform;
            tf_transform = tf_buffer_->lookupTransform(
                global_frame_,
                robot_base_frame_,
                tf2::TimePointZero
            );

            Eigen::Vector2f distance_of_goal;
            distance_of_goal.x() = goal_position_.x() - tf_transform.transform.translation.x;
            distance_of_goal.y() = goal_position_.y() - tf_transform.transform.translation.y;

            std_msgs::msg::Bool msg;

            if(distance_of_goal.lpNorm<2>() < current_goal_radius_) {
                RCLCPP_INFO(this->get_logger(), "Goal reached from goal_event_handler::radius_node");
                msg.data = true;
            }
            else {
                msg.data = false;
            }

            is_reached_goal_publisher_->publish(msg);
        }
        catch(const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Transform failed %s to %s: %s", 
                                 global_frame_.c_str(), robot_base_frame_.c_str(), e.what());
        }
    }
};

auto main(int argc, char **argv) -> int {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RadiusNode>();
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "Finish radius_node");
    rclcpp::shutdown();

    return 0;
}

