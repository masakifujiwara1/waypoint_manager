
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <waypoint_manager_msgs/msg/waypoint.hpp>

class Node : public rclcpp::Node {
    public :
        Node();

    private :
        bool latch_;

        std::string pose_topic_,
                    waypoint_topic_,
                    frame_id_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
        rclcpp::Subscription<waypoint_manager_msgs::msg::Waypoint>::SharedPtr waypoint_subscriber_;

        void waypointCallback(const waypoint_manager_msgs::msg::Waypoint::SharedPtr msg);
};

Node::Node() : rclcpp::Node("waypoint_to_posestamped_node") {
    this->declare_parameter("latch", false);
    this->declare_parameter("frame_id", "map");
    this->declare_parameter("pose_topic", "move_base_simple/goal");
    this->declare_parameter("waypoint_topic", "waypoint");

    latch_ = this->get_parameter("latch").as_bool();
    frame_id_ = this->get_parameter("frame_id").as_string();
    pose_topic_ = this->get_parameter("pose_topic").as_string();
    waypoint_topic_ = this->get_parameter("waypoint_topic").as_string();

    // QoS settings for latch behavior (transient local)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    if (latch_) {
        qos.transient_local();
    }

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        pose_topic_,
        qos
    );
    waypoint_subscriber_ = this->create_subscription<waypoint_manager_msgs::msg::Waypoint>(
        waypoint_topic_,
        10,
        std::bind(&Node::waypointCallback, this, std::placeholders::_1)
    );
}

void Node::waypointCallback(const waypoint_manager_msgs::msg::Waypoint::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped pose_stamped;

    pose_stamped.pose = msg->pose;
    pose_stamped.header.frame_id = frame_id_;
    pose_stamped.header.stamp = this->now();

    pose_publisher_->publish(pose_stamped);
}

auto main(int argc, char **argv) -> int {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
