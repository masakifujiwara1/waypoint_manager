#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <waypoint_manager_msgs/msg/waypoints.hpp>
#include <waypoint_manager_msgs/msg/route.hpp>
#include <waypoint_manager_msgs/msg/waypoint_stamped.hpp>

struct Parameters {
    bool latch,
         enable_2d;

    int publish_queue_size,
        subscribe_queue_size;

    float default_goal_radius,
         set_goal_radius;

    std::string pose_array_topic,
                update_waypoint_topic,
                erase_waypoint_topic,
                append_route_topic,
                erase_route_topic,
                insert_route_topic,
                waypoints_topic,
                route_topic,
                route_path_topic;
};

class Node : public rclcpp::Node {
    public:
        Node();

        void spin();

    private:
        Parameters param;

        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_publisher;
        rclcpp::Publisher<waypoint_manager_msgs::msg::WaypointStamped>::SharedPtr update_waypoint_publisher;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr erase_waypoint_publisher;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr append_route_publisher;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr erase_route_publisher;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr insert_route_publisher;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_path_publisher;

        rclcpp::Subscription<waypoint_manager_msgs::msg::Route>::SharedPtr route_subscriber;
        rclcpp::Subscription<waypoint_manager_msgs::msg::Waypoints>::SharedPtr waypoints_subscriber;

        std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_server;
        interactive_markers::MenuHandler menu_handler;

        std::unique_ptr<waypoint_manager_msgs::msg::Route> current_route_msg;
        std::unique_ptr<waypoint_manager_msgs::msg::Waypoints> current_waypoints_msg;

        void waypointsCallback(const waypoint_manager_msgs::msg::Waypoints::SharedPtr msg);
        void routeCallback(const waypoint_manager_msgs::msg::Route::SharedPtr msg);
        void waypointFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback);
        void deleteWaypointFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback);
        void appendRouteFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback);
        void deleteRouteFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback);
        void insertRouteFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback);
        void switchStopPointFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback);
        void SetGoalRadiusFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback);
};

Node::Node() : Node("waypoint_visualization_node") {
    this->declare_parameter<bool>("latch", false);
    this->declare_parameter<bool>("enable_2d", true);
    this->declare_parameter<int>("publish_queue_size", 2);
    this->declare_parameter<int>("subscribe_queue_size", 2);
    this->declare_parameter<float>("default_goal_radius", 1.0f);
    this->declare_parameter<std::string>("pose_array_topic", "waypoint/array");
    this->declare_parameter<std::string>("update_waypoint_topic", "waypoint/update");
    this->declare_parameter<std::string>("erase_waypoint_topic", "waypoint/erase");
    this->declare_parameter<std::string>("waypoints_topic", "waypoints");
    this->declare_parameter<std::string>("route_topic", "route");
    this->declare_parameter<std::string>("append_route_topic", "route/append");
    this->declare_parameter<std::string>("erase_route_topic", "route/erase");
    this->declare_parameter<std::string>("insert_route_topic", "route/insert");
    this->declare_parameter<std::string>("route_path_topic", "route/path");
    this->declare_parameter<float>("set_goal_radius", 1.0f);

    this->get_parameter("latch", param.latch);
    this->get_parameter("enable_2d", param.enable_2d);
    this->get_parameter("publish_queue_size", param.publish_queue_size);
    this->get_parameter("subscribe_queue_size", param.subscribe_queue_size);
    this->get_parameter("default_goal_radius", param.default_goal_radius);
    this->get_parameter("pose_array_topic", param.pose_array_topic);
    this->get_parameter("update_waypoint_topic", param.update_waypoint_topic);
    this->get_parameter("erase_waypoint_topic", param.erase_waypoint_topic);
    this->get_parameter("waypoints_topic", param.waypoints_topic);
    this->get_parameter("route_topic", param.route_topic);
    this->get_parameter("append_route_topic", param.append_route_topic);
    this->get_parameter("erase_route_topic", param.erase_route_topic);
    this->get_parameter("insert_route_topic", param.insert_route_topic);
    this->get_parameter("route_path_topic", param.route_path_topic);
    this->get_parameter("set_goal_radius", param.set_goal_radius);

    pose_array_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>(param.pose_array_topic, param.publish_queue_size);
    update_waypoint_publisher = this->create_publisher<waypoint_manager_msgs::msg::WaypointStamped>(param.update_waypoint_topic, param.publish_queue_size);
    erase_waypoint_publisher = this->create_publisher<std_msgs::msg::String>(param.erase_waypoint_topic, param.publish_queue_size);
    append_route_publisher = this->create_publisher<std_msgs::msg::String>(param.append_route_topic, param.publish_queue_size);
    erase_route_publisher = this->create_publisher<std_msgs::msg::String>(param.erase_route_topic, param.publish_queue_size);
    insert_route_publisher = this->create_publisher<std_msgs::msg::String>(param.insert_route_topic, param.publish_queue_size);
    route_path_publisher = this->create_publisher<nav_msgs::msg::Path>(param.route_path_topic, param.publish_queue_size);

    route_subscriber = this->create_subscription<waypoint_manager_msgs::msg::Route>(param.route_topic, param.subscribe_queue_size, std::bind(&Node::routeCallback, this, std::placeholders::_1));
    waypoints_subscriber = this->create_subscription<waypoint_manager_msgs::msg::Waypoints>(param.waypoints_topic, param.subscribe_queue_size, std::bind(&Node::waypointsCallback, this, std::placeholders::_1));

    interactive_server = std::make_shared<interactive_markers::InteractiveMarkerServer>("waypoint_visualization_node", this);

    menu_handler.insert("Append route", std::bind(&Node::appendRouteFeedback, this, std::placeholders::_1));
    menu_handler.insert("Insert route", std::bind(&Node::insertRouteFeedback, this, std::placeholders::_1));
    menu_handler.insert("Delete route", std::bind(&Node::deleteRouteFeedback, this, std::placeholders::_1));
    menu_handler.insert("Delete waypoint", std::bind(&Node::deleteWaypointFeedback, this, std::placeholders::_1));
    auto properties_menu_id = menu_handler.insert("Properties");
    auto stop_point_menu_id = menu_handler.insert(properties_menu_id, "Stop point", std::bind(&Node::switchStopPointFeedback, this, std::placeholders::_1));
    menu_handler.setCheckState(stop_point_menu_id, interactive_markers::MenuHandler::UNCHECKED);

    auto set_goal_radius_menu_id = menu_handler.insert(properties_menu_id, "Set goal radius", std::bind(&Node::SetGoalRadiusFeedback, this, std::placeholders::_1));
    menu_handler.setCheckState(set_goal_radius_menu_id, interactive_markers::MenuHandler::UNCHECKED);
}

void Node::spin() {
    RCLCPP_INFO(this->get_logger(), "Start spin in waypoint_visualization_node");
    rclcpp::spin(this->shared_from_this());
}

void Node::waypointsCallback(const waypoint_manager_msgs::msg::Waypoints::SharedPtr msg) {
    geometry_msgs::msg::PoseArray pose_array;

    for(const auto &wp : msg->waypoints) {
        pose_array.poses.push_back(wp.pose);
    }
    pose_array.header.frame_id = msg->info.header.frame_id;
    pose_array.header.stamp = this->now();
    pose_array_publisher->publish(pose_array);

    interactive_server->clear();
    std::vector<visualization_msgs::msg::InteractiveMarker> interactive_marker_msgs;

    for(const auto &wp : msg->waypoints) {
        visualization_msgs::msg::InteractiveMarker interactive_marker_msg;

        visualization_msgs::msg::InteractiveMarkerControl waypoint_control,
                                                           orientation_control;

        waypoint_control.always_visible = true;
        orientation_control.always_visible = true;
        interactive_marker_msg.pose = wp.pose;
        
        if(param.enable_2d) {
            waypoint_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
            waypoint_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
            orientation_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
            orientation_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;

            interactive_marker_msg.pose.position.z = 0;
            interactive_marker_msg.pose.orientation.x = 0;
            interactive_marker_msg.pose.orientation.y = 0;

            tf2::Quaternion orientation{0, 1, 0, 1};

            orientation.normalize();
            tf2::quaternionTFToMsg(orientation, waypoint_control.orientation);
            tf2::quaternionTFToMsg(orientation, orientation_control.orientation);
        }
        else {
            waypoint_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
            waypoint_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
            orientation_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D;
            orientation_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;

            tf2::Quaternion orientation{0, 1, 0, 1};

            orientation.normalize();
            tf2::quaternionTFToMsg(orientation, waypoint_control.orientation);
            tf2::quaternionTFToMsg(orientation, orientation_control.orientation);
        }

        visualization_msgs::msg::Marker arrow_marker;
        std::vector<visualization_msgs::msg::Marker> flag_marker_parts;

        bool stop_flag = false;
        float goal_radius = param.default_goal_radius;

        for(const auto &p : wp.properties) {
            if(p.name == "goal_radius") {
                goal_radius = std::stof(p.data);
            }
            if(p.name == "stop" && p.data == "true") {
                stop_flag = true;
            }
        }

        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.scale.x = 1;
        arrow_marker.scale.y = 0.1;
        arrow_marker.scale.z = 0.1;
        arrow_marker.color.r = 1;
        arrow_marker.color.g = 0;
        arrow_marker.color.b = 0;
        arrow_marker.color.a = 0.7;

        if(goal_radius > 0) {
            flag_marker_parts.resize(5);

            flag_marker_parts[4].type = visualization_msgs::msg::Marker::CYLINDER;
            flag_marker_parts[4].scale.x = 2 * goal_radius;
            flag_marker_parts[4].scale.y = 2 * goal_radius;
            flag_marker_parts[4].scale.z = 0.01;

            if(stop_flag) {
                flag_marker_parts[4].color.r = 0.7;
                flag_marker_parts[4].color.g = 0;
                flag_marker_parts[4].color.b = 0;
            }
            else {
                flag_marker_parts[4].color.r = 0;
                flag_marker_parts[4].color.g = 1;
                flag_marker_parts[4].color.b = 0;
            }
        }
        else {
            flag_marker_parts.resize(4);
        }
        flag_marker_parts[0].type = visualization_msgs::msg::Marker::CYLINDER;
        flag_marker_parts[1].type = visualization_msgs::msg::Marker::CUBE;
        flag_marker_parts[2].type = visualization_msgs::msg::Marker::SPHERE;
        flag_marker_parts[3].type = visualization_msgs::msg::Marker::CYLINDER;
        flag_marker_parts[0].scale.x = 0.1;
        flag_marker_parts[0].scale.y = 0.1;
        flag_marker_parts[0].scale.z = 1.2;
        flag_marker_parts[1].scale.x = 0.001;
        flag_marker_parts[1].scale.y = 0.6;
        flag_marker_parts[1].scale.z = 0.4;
        flag_marker_parts[2].scale.x = 0.2;
        flag_marker_parts[2].scale.y = 0.2;
        flag_marker_parts[2].scale.z = 0.2;
        flag_marker_parts[3].scale.x = 0.9;
        flag_marker_parts[3].scale.y = 0.9;
        flag_marker_parts[3].scale.z = 1.5;
        flag_marker_parts[0].pose.position.x = 0;
        flag_marker_parts[0].pose.position.y = 0;
        flag_marker_parts[0].pose.position.z = flag_marker_parts[0].scale.z * 0.5;
        flag_marker_parts[1].pose.position.x = 0;
        flag_marker_parts[1].pose.position.y = 0.3;
        flag_marker_parts[1].pose.position.z = 1.0;
        flag_marker_parts[2].pose.position.x = 0;
        flag_marker_parts[2].pose.position.y = 0;
        flag_marker_parts[2].pose.position.z = 1.25;
        flag_marker_parts[3].pose.position.z = flag_marker_parts[3].scale.z * 0.5;
        flag_marker_parts[0].color.r = 0.75;
        flag_marker_parts[0].color.g = 0.75;
        flag_marker_parts[0].color.b = 0.75;
        flag_marker_parts[1].color.r = 1;
        flag_marker_parts[1].color.g = 1;
        flag_marker_parts[1].color.b = 1;
        flag_marker_parts[2].color.r = 1;
        flag_marker_parts[2].color.g = 1;
        flag_marker_parts[2].color.b = 0;

        for(auto &&fmp : flag_marker_parts) {
            fmp.color.a = 0.6;
        }
        flag_marker_parts[3].color.a = 0.0;

        for(const auto &fmp : flag_marker_parts) {
            waypoint_control.markers.push_back(fmp);
        }
        orientation_control.markers.push_back(arrow_marker);

        interactive_marker_msg.controls.push_back(waypoint_control);
        interactive_marker_msg.controls.push_back(orientation_control);

        interactive_marker_msg.header.frame_id = msg->info.header.frame_id;
        interactive_marker_msg.name = wp.identity;
        interactive_marker_msg.description = wp.identity;
        interactive_marker_msg.header.stamp = this->now();

        interactive_marker_msgs.push_back(interactive_marker_msg);
    }
    current_waypoints_msg = std::make_unique<waypoint_manager_msgs::msg::Waypoints>(*msg);

    if(!current_route_msg) {
        RCLCPP_WARN(this->get_logger(), "No sync route path");
        for(const auto &im_msg : interactive_marker_msgs) {
            interactive_server->insert(im_msg, std::bind(&Node::waypointFeedback, this, std::placeholders::_1));
            menu_handler.apply(*interactive_server, im_msg.name);
        }
        interactive_server->applyChanges();
        return;
    }
    nav_msgs::msg::Path route_path_msg;

    constexpr auto text_marker_scale = 1.0;
    int route_counter = 0;

    std::unordered_map<std::string, int> count_of_hit_marker;
    for(const auto &identity : current_route_msg->identities) {
        visualization_msgs::msg::InteractiveMarkerControl route_count_control;
        visualization_msgs::msg::Marker route_count_marker;

        route_count_control.always_visible = true;
        route_count_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        route_count_marker.color.r = 1;
        route_count_marker.color.g = 1;
        route_count_marker.color.b = 1;
        route_count_marker.color.a = 1;
        route_count_marker.scale.x = text_marker_scale;
        route_count_marker.scale.y = text_marker_scale;
        route_count_marker.scale.z = text_marker_scale;

        for(auto &&im_msg : interactive_marker_msgs) {
            if(identity == im_msg.name) {
                geometry_msgs::msg::PoseStamped waypoint_pose_msg;

                waypoint_pose_msg.pose = im_msg.pose;
                waypoint_pose_msg.header.frame_id = msg->info.header.frame_id;
                waypoint_pose_msg.header.stamp = this->now();
                
                route_path_msg.poses.push_back(waypoint_pose_msg);

                route_count_marker.text += std::to_string(route_counter) + "\n";
                for(int i = 0; i < count_of_hit_marker[identity]; i ++) {
                    route_count_marker.text += "\n\n";
                }
                route_count_control.markers.push_back(route_count_marker);
                im_msg.controls.push_back(route_count_control);
                count_of_hit_marker[identity] ++;
            }
        }
        route_counter ++;
    }
    for(const auto &im_msg : interactive_marker_msgs) {
        interactive_server->insert(im_msg, std::bind(&Node::waypointFeedback, this, std::placeholders::_1));
        menu_handler.apply(*interactive_server, im_msg.name);
    }
    interactive_server->applyChanges();
    route_path_msg.header.frame_id = msg->info.header.frame_id;
    route_path_msg.header.stamp = this->now();
    route_path_publisher->publish(route_path_msg);
    current_route_msg.reset();
}

void Node::routeCallback(const waypoint_manager_msgs::msg::Route::SharedPtr msg) {
    current_route_msg = std::make_unique<waypoint_manager_msgs::msg::Route>(*msg);
}

void Node::waypointFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback) {
    if(feedback->mouse_point_valid == 1) {
        waypoint_manager_msgs::msg::WaypointStamped msg;

        msg.header.stamp = this->now();
        msg.header.frame_id = feedback->header.frame_id;
        msg.waypoint.identity = feedback->marker_name;
        msg.waypoint.pose = feedback->pose;

        update_waypoint_publisher->publish(msg);
    }
    interactive_server->setPose(feedback->marker_name, feedback->pose);
    interactive_server->applyChanges();
}

void Node::deleteWaypointFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback) {
    RCLCPP_INFO(this->get_logger(), "Called deleteWaypointFeedback %s", feedback->marker_name.c_str());

    std_msgs::msg::String msg;

    msg.data = feedback->marker_name;
    erase_waypoint_publisher->publish(msg);
}

void Node::appendRouteFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback) {
    RCLCPP_INFO(this->get_logger(), "Called appendRouteFeedback %s", feedback->marker_name.c_str());

    std_msgs::msg::String msg;

    msg.data = feedback->marker_name;
    append_route_publisher->publish(msg);
}

void Node::deleteRouteFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback) {
    RCLCPP_INFO(this->get_logger(), "Called deleteWaypointFeedback %s", feedback->marker_name.c_str());

    std_msgs::msg::String msg;

    msg.data = feedback->marker_name;
    erase_route_publisher->publish(msg);
}

void Node::insertRouteFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback) {
    RCLCPP_INFO(this->get_logger(), "Called insertRouteFeedback %s", feedback->marker_name.c_str());

    std_msgs::msg::String msg;

    msg.data = feedback->marker_name;
    insert_route_publisher->publish(msg);
}

void Node::switchStopPointFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback) {
    RCLCPP_INFO(this->get_logger(), "Called switchStopPointFeedback %s", feedback->marker_name.c_str());

    interactive_markers::MenuHandler::EntryHandle entry_menu_id = feedback->menu_entry_id;
    interactive_markers::MenuHandler::CheckState menu_check_state;
    waypoint_manager_msgs::msg::WaypointStamped msg;

    menu_handler.getCheckState(entry_menu_id, menu_check_state);

    if(menu_check_state == interactive_markers::MenuHandler::CHECKED) {
        menu_handler.setCheckState(entry_menu_id, interactive_markers::MenuHandler::UNCHECKED);

        msg.waypoint.properties.push_back(waypoint_manager_msgs::msg::Property());
        msg.waypoint.properties.back().name = "stop";
        msg.waypoint.properties.back().data = "false";
    }
    else if(menu_check_state == interactive_markers::MenuHandler::UNCHECKED) {
        menu_handler.setCheckState(entry_menu_id, interactive_markers::MenuHandler::CHECKED);

        msg.waypoint.properties.push_back(waypoint_manager_msgs::msg::Property());
        msg.waypoint.properties.back().name = "stop";
        msg.waypoint.properties.back().data = "true";
    }
    else {
        menu_handler.setCheckState(entry_menu_id, interactive_markers::MenuHandler::UNCHECKED);
    }
    msg.waypoint.pose = feedback->pose;
    msg.waypoint.identity = feedback->marker_name;
    msg.header.frame_id = feedback->header.frame_id;
    msg.header.stamp = this->now();
    update_waypoint_publisher->publish(msg);

    menu_handler.reApply(*interactive_server);
    interactive_server->applyChanges();
}

void Node::SetGoalRadiusFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback) {
    RCLCPP_INFO(this->get_logger(), "Called SetGoalRadiusFeedback %s", feedback->marker_name.c_str());

    this->get_parameter("set_goal_radius", param.set_goal_radius);

    interactive_markers::MenuHandler::EntryHandle entry_menu_id = feedback->menu_entry_id;
    interactive_markers::MenuHandler::CheckState menu_check_state;
    waypoint_manager_msgs::msg::WaypointStamped msg;

    menu_handler.getCheckState(entry_menu_id, menu_check_state);

    if(menu_check_state == interactive_markers::MenuHandler::CHECKED) {
        menu_handler.setCheckState(entry_menu_id, interactive_markers::MenuHandler::UNCHECKED);

        msg.waypoint.properties.push_back(waypoint_manager_msgs::msg::Property());
        msg.waypoint.properties.back().name = "goal_radius";
        msg.waypoint.properties.back().data = "1.0";
    }
    else if(menu_check_state == interactive_markers::MenuHandler::UNCHECKED) {
        menu_handler.setCheckState(entry_menu_id, interactive_markers::MenuHandler::CHECKED);

        msg.waypoint.properties.push_back(waypoint_manager_msgs::msg::Property());
        msg.waypoint.properties.back().name = "goal_radius";
        RCLCPP_INFO(this->get_logger(), "Called SetGoalRadiusFeedback %f", param.set_goal_radius);
        msg.waypoint.properties.back().data = std::to_string(param.set_goal_radius);
    }
    else {
        menu_handler.setCheckState(entry_menu_id, interactive_markers::MenuHandler::UNCHECKED);
    }
    msg.waypoint.pose = feedback->pose;
    msg.waypoint.identity = feedback->marker_name;
    msg.header.frame_id = feedback->header.frame_id;
    msg.header.stamp = this->now();
    update_waypoint_publisher->publish(msg);

    menu_handler.reApply(*interactive_server);
    interactive_server->applyChanges();
}

auto main(int argc, char **argv) -> int {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Node>();
    node->spin();

    rclcpp::shutdown();

    return 0;
}
