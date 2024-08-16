#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <waypoint_manager_msgs/msg/waypoint.hpp>

class Node : public rclcpp::Node {
public:
    Node();

private:
    bool latch;
    std::string pose_topic, waypoint_topic, frame_id;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Subscription<waypoint_manager_msgs::msg::Waypoint>::SharedPtr waypoint_subscriber;

    void waypointCallback(const waypoint_manager_msgs::msg::Waypoint::SharedPtr msg);
};

Node::Node() : Node("waypoint_to_posestamped_node") {
    this->declare_parameter("latch", false);
    this->declare_parameter("frame_id", "map");
    this->declare_parameter("pose_topic", "move_base_simple/goal");
    this->declare_parameter("waypoint_topic", "waypoint");

    this->get_parameter("latch", latch);
    this->get_parameter("frame_id", frame_id);
    this->get_parameter("pose_topic", pose_topic);
    this->get_parameter("waypoint_topic", waypoint_topic);

    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, rclcpp::QoS(1).transient_local(latch));
    waypoint_subscriber = this->create_subscription<waypoint_manager_msgs::msg::Waypoint>(
        waypoint_topic,
        rclcpp::QoS(1),
        std::bind(&Node::waypointCallback, this, std::placeholders::_1)
    );
}

void Node::waypointCallback(const waypoint_manager_msgs::msg::Waypoint::SharedPtr msg) {
    auto pose_stamped = geometry_msgs::msg::PoseStamped();

    pose_stamped.pose = msg->pose;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = this->now();

    pose_publisher->publish(pose_stamped);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
