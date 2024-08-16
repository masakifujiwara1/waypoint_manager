#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>

#include <Eigen/Dense>

#include <waypoint_manager_msgs/msg/waypoint.hpp>
#include <waypoint_manager_msgs/msg/waypoint_stamped.hpp>
#include <waypoint_manager_msgs/msg/waypoints.hpp>
#include <waypoint_manager_msgs/msg/route.hpp>

#include <waypoint_server/waypoint_server.hpp>

namespace waypoint_server {
    struct NodeParameters {
        std::string goal_topic,
                    waypoint_topic,
                    is_reached_goal_topic,
                    regist_goal_pose_topic,
                    regist_goal_point_topic,
                    erase_goal_topic,
                    update_goal_topic,
                    route_topic,
                    append_route_topic,
                    erase_route_topic,
                    insert_route_topic,
                    waypoints_topic,
                    clear_costmap_srv;

        std::string regist_waypoint_prefix;

        std::string robot_base_frame,
                    global_frame;

        std::string waypoints_file,
                    route_file;

        bool debug,
             latch,
             enable_2d,
             enable_3d,
             enable_loop;

        int publish_queue_size,
            subscribe_queue_size;

        float wait_publish_waypoints_time;

        double goal_publish_frequency;
    };

    class Node : public rclcpp::Node {
    public:
        Node();

        void spin();

    private:
        rclcpp::Publisher<waypoint_manager_msgs::msg::Waypoint>::SharedPtr waypoint_publisher;
        rclcpp::Publisher<waypoint_manager_msgs::msg::Waypoints>::SharedPtr waypoints_publisher;
        rclcpp::Publisher<waypoint_manager_msgs::msg::Route>::SharedPtr route_publisher;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_reached_goal_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr regist_goal_pose_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr regist_goal_point_subscriber;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr erase_goal_subscriber;
        rclcpp::Subscription<waypoint_manager_msgs::msg::WaypointStamped>::SharedPtr update_goal_subscriber;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr append_route_subscriber;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr erase_route_subscriber;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr insert_route_subscriber;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_waypoints_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_route_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_route_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr switch_cancel_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr next_waypoint_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prev_waypoint_service;

        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmap_service;

        NodeParameters param;

        Map waypoint_map;
        Route router;

        std::atomic<unsigned int> regist_goal_id;
        std::atomic_bool is_cancel;

        void isReachedGoal(const std_msgs::msg::Bool::SharedPtr msg);
        void registGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void registGoalPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
        void eraseGoal(const std_msgs::msg::String::SharedPtr msg);
        void updateGoalPose(const waypoint_manager_msgs::msg::WaypointStamped::SharedPtr msg);
        void appendRoute(const std_msgs::msg::String::SharedPtr msg);
        void eraseRoute(const std_msgs::msg::String::SharedPtr msg);
        void insertRoute(const std_msgs::msg::String::SharedPtr msg);

        bool save(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
        bool saveWaypoints(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
        bool saveRoute(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
        bool resetRoute(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
        bool switchCancel(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
        bool nextWaypoint(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
        bool prevWaypoint(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

        void publishGoal();
        void publishWaypoints();
        void publishRoute();
        void publishLatchedData();
        void exchangeCancelState();
    };

    Node::Node() : Node("waypoint_server_node") {
        // ROS2におけるパラメータの取得
        this->declare_parameter("goal_topic", "move_base_simple/goal");
        this->declare_parameter("waypoint_topic", "waypoint");
        this->declare_parameter("is_reached_goal_topic", "waypoint/is_reached");
        this->declare_parameter("regist_goal_pose_topic", "waypoint/regist_pose");
        this->declare_parameter("regist_goal_point_topic", "waypoint/regist_point");
        this->declare_parameter("erase_goal_topic", "waypoint/erase");
        this->declare_parameter("update_goal_topic", "waypoint/update");
        this->declare_parameter("route_topic", "route");
        this->declare_parameter("append_route_topic", "route/append");
        this->declare_parameter("erase_route_topic", "route/erase");
        this->declare_parameter("insert_route_topic", "route/insert");
        this->declare_parameter("waypoints_topic", "waypoints");
        this->declare_parameter("regist_waypoint_prefix", "registed_");
        this->declare_parameter("robot_base_frame", "base_link");
        this->declare_parameter("global_frame", "map");
        this->declare_parameter("waypoints_file", "");
        this->declare_parameter("route_file", "");
        this->declare_parameter("debug", false);
        this->declare_parameter("latch", false);
        this->declare_parameter("enable_loop", false);
        this->declare_parameter("publish_queue_size", 1);
        this->declare_parameter("subscribe_queue_size", 1);
        this->declare_parameter("wait_publish_waypoints_time", 5e-3f);
        this->declare_parameter("goal_publish_frequency", 0.5);
        this->declare_parameter("clear_costmap_srv", "move_base/clear_costmaps");

        this->get_parameter("goal_topic", param.goal_topic);
        this->get_parameter("waypoint_topic", param.waypoint_topic);
        this->get_parameter("is_reached_goal_topic", param.is_reached_goal_topic);
        this->get_parameter("regist_goal_pose_topic", param.regist_goal_pose_topic);
        this->get_parameter("regist_goal_point_topic", param.regist_goal_point_topic);
        this->get_parameter("erase_goal_topic", param.erase_goal_topic);
        this->get_parameter("update_goal_topic", param.update_goal_topic);
        this->get_parameter("route_topic", param.route_topic);
        this->get_parameter("append_route_topic", param.append_route_topic);
        this->get_parameter("erase_route_topic", param.erase_route_topic);
        this->get_parameter("insert_route_topic", param.insert_route_topic);
        this->get_parameter("waypoints_topic", param.waypoints_topic);
        this->get_parameter("regist_waypoint_prefix", param.regist_waypoint_prefix);
        this->get_parameter("robot_base_frame", param.robot_base_frame);
        this->get_parameter("global_frame", param.global_frame);
        this->get_parameter("waypoints_file", param.waypoints_file);
        this->get_parameter("route_file", param.route_file);
        this->get_parameter("debug", param.debug);
        this->get_parameter("latch", param.latch);
        this->get_parameter("enable_loop", param.enable_loop);
        this->get_parameter("publish_queue_size", param.publish_queue_size);
        this->get_parameter("subscribe_queue_size", param.subscribe_queue_size);
        this->get_parameter("wait_publish_waypoints_time", param.wait_publish_waypoints_time);
        this->get_parameter("goal_publish_frequency", param.goal_publish_frequency);
        this->get_parameter("clear_costmap_srv", param.clear_costmap_srv);

        waypoint_publisher = this->create_publisher<waypoint_manager_msgs::msg::Waypoint>(param.waypoint_topic, param.publish_queue_size);
        waypoints_publisher = this->create_publisher<waypoint_manager_msgs::msg::Waypoints>(param.waypoints_topic, param.publish_queue_size);
        route_publisher = this->create_publisher<waypoint_manager_msgs::msg::Route>(param.route_topic, param.publish_queue_size);

        is_reached_goal_subscriber = this->create_subscription<std_msgs::msg::Bool>(
            param.is_reached_goal_topic,
            param.subscribe_queue_size,
            std::bind(&Node::isReachedGoal, this, std::placeholders::_1)
        );

        regist_goal_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            param.regist_goal_pose_topic,
            param.subscribe_queue_size,
            std::bind(&Node::registGoalPose, this, std::placeholders::_1)
        );

        regist_goal_point_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(
            param.regist_goal_point_topic,
            param.subscribe_queue_size,
            std::bind(&Node::registGoalPoint, this, std::placeholders::_1)
        );

        erase_goal_subscriber = this->create_subscription<std_msgs::msg::String>(
            param.erase_goal_topic,
            param.subscribe_queue_size,
            std::bind(&Node::eraseGoal, this, std::placeholders::_1)
        );

        update_goal_subscriber = this->create_subscription<waypoint_manager_msgs::msg::WaypointStamped>(
            param.update_goal_topic,
            param.subscribe_queue_size,
            std::bind(&Node::updateGoalPose, this, std::placeholders::_1)
        );

        append_route_subscriber = this->create_subscription<std_msgs::msg::String>(
            param.append_route_topic,
            param.subscribe_queue_size,
            std::bind(&Node::appendRoute, this, std::placeholders::_1)
        );

        erase_route_subscriber = this->create_subscription<std_msgs::msg::String>(
            param.erase_route_topic,
            param.subscribe_queue_size,
            std::bind(&Node::eraseRoute, this, std::placeholders::_1)
        );

        insert_route_subscriber = this->create_subscription<std_msgs::msg::String>(
            param.insert_route_topic,
            param.subscribe_queue_size,
            std::bind(&Node::insertRoute, this, std::placeholders::_1)
        );

        save_service = this->create_service<std_srvs::srv::Trigger>(
            "save",
            std::bind(&Node::save, this, std::placeholders::_1, std::placeholders::_2)
        );

        save_waypoints_service = this->create_service<std_srvs::srv::Trigger>(
            "save_waypoints",
            std::bind(&Node::saveWaypoints, this, std::placeholders::_1, std::placeholders::_2)
        );

        save_route_service = this->create_service<std_srvs::srv::Trigger>(
            "save_route",
            std::bind(&Node::saveRoute, this, std::placeholders::_1, std::placeholders::_2)
        );

        reset_route_service = this->create_service<std_srvs::srv::Trigger>(
            "reset_route",
            std::bind(&Node::resetRoute, this, std::placeholders::_1, std::placeholders::_2)
        );

        switch_cancel_service = this->create_service<std_srvs::srv::Trigger>(
            "switch_cancel",
            std::bind(&Node::switchCancel, this, std::placeholders::_1, std::placeholders::_2)
        );

        next_waypoint_service = this->create_service<std_srvs::srv::Trigger>(
            "next_waypoint",
            std::bind(&Node::nextWaypoint, this, std::placeholders::_1, std::placeholders::_2)
        );

        prev_waypoint_service = this->create_service<std_srvs::srv::Trigger>(
            "prev_waypoint",
            std::bind(&Node::prevWaypoint, this, std::placeholders::_1, std::placeholders::_2)
        );

        clear_costmap_service = this->create_client<std_srvs::srv::Empty>(param.clear_costmap_srv);

        is_cancel.store(true);
        regist_goal_id.store(0);

        RCLCPP_INFO(this->get_logger(), "Loading waypoints_file %s", param.waypoints_file.c_str());
        waypoint_map.load(param.waypoints_file);
        RCLCPP_INFO(this->get_logger(), "Success load waypoints_file");

        RCLCPP_INFO(this->get_logger(), "Loading route_file: %s", param.route_file.c_str());
        router.load(param.route_file);
        router.loop(param.enable_loop);
        RCLCPP_INFO(this->get_logger(), "Success load route_file");
        RCLCPP_INFO(this->get_logger(), "Count of skip ids %d", router.getSkipIds());
    }

    void Node::spin() {
        rclcpp::Rate rate(param.goal_publish_frequency);
        publishLatchedData();

        while (rclcpp::ok()) {
            publishGoal();
            rate.sleep();
        }
    }

    void Node::isReachedGoal(const std_msgs::msg::Bool::SharedPtr msg) {
        if (router.isEmpty()) {
            RCLCPP_WARN(this->get_logger(), "Route size of zero");
            return;
        }
        if (is_cancel.load()) {
            return;
        }
        if (!msg->data) {
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Goal reached %s from waypoint_server_node", router.getIndex().c_str());
        if (waypoint_map[router.getIndex()].properties["stop"] == "true") {
            RCLCPP_INFO(this->get_logger(), "Current waypoint properties stop is true");
            RCLCPP_INFO(this->get_logger(), "Please call the ~/next_waypoint service");
            return;
        }
        if (!router.forwardIndex()) {
            RCLCPP_WARN(this->get_logger(), "Not found next index for route");
            is_cancel.store(true);
        } else {
            publishGoal();
        }
    }

    Map::Key generateKey(const std::atomic<unsigned int> &id, const std::string &prefix) {
        return prefix + std::to_string(id.load()) + "_" + std::to_string(rclcpp::Clock().now().nanoseconds());
    }

    void Node::registGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const auto name = generateKey(regist_goal_id, param.regist_waypoint_prefix);

        waypoint_map[name].goal.x() = msg->pose.position.x;
        waypoint_map[name].goal.y() = msg->pose.position.y;
        waypoint_map[name].goal.z() = msg->pose.position.z;
        waypoint_map[name].quaternion.x() = msg->pose.orientation.x;
        waypoint_map[name].quaternion.y() = msg->pose.orientation.y;
        waypoint_map[name].quaternion.z() = msg->pose.orientation.z;
        waypoint_map[name].quaternion.w() = msg->pose.orientation.w;
        waypoint_map.setQuaternion(name);

        RCLCPP_INFO(this->get_logger(), "Add waypoint %s from pose", name.c_str());

        regist_goal_id.store(regist_goal_id.load() + 1);

        publishLatchedData();
    }

    void Node::registGoalPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        const auto name = generateKey(regist_goal_id, param.regist_waypoint_prefix);

        waypoint_map[name].goal.x() = msg->point.x;
        waypoint_map[name].goal.y() = msg->point.y;
        waypoint_map[name].goal.z() = msg->point.z;
        waypoint_map[name].quaternion.x() = 0;
        waypoint_map[name].quaternion.y() = 0;
        waypoint_map[name].quaternion.z() = 0;
        waypoint_map[name].quaternion.w() = 1;
        waypoint_map.setQuaternion(name);

        RCLCPP_INFO(this->get_logger(), "Add waypoint %s from point", name.c_str());

        regist_goal_id.store(regist_goal_id.load() + 1);

        publishLatchedData();
    }

    void Node::eraseGoal(const std_msgs::msg::String::SharedPtr msg) {
        if (!waypoint_map.hasKey(msg->data)) {
            RCLCPP_INFO(this->get_logger(), "Do not have waypoint id %s", msg->data.c_str());
            return;
        }
        waypoint_map.erase(msg->data);
        router.erase(msg->data);

        RCLCPP_INFO(this->get_logger(), "Removed waypoint id %s", msg->data.c_str());

        publishLatchedData();
    }

    void Node::updateGoalPose(const waypoint_manager_msgs::msg::WaypointStamped::SharedPtr msg) {
        if (param.global_frame != msg->header.frame_id) {
            RCLCPP_WARN(this->get_logger(), "The frame_id is different, so the goal pose is not updated");
            return;
        }
        decltype(auto) name = msg->waypoint.identity;

        waypoint_map[name].goal.x() = msg->waypoint.pose.position.x;
        waypoint_map[name].goal.y() = msg->waypoint.pose.position.y;
        waypoint_map[name].goal.z() = msg->waypoint.pose.position.z;
        waypoint_map[name].quaternion.x() = msg->waypoint.pose.orientation.x;
        waypoint_map[name].quaternion.y() = msg->waypoint.pose.orientation.y;
        waypoint_map[name].quaternion.z() = msg->waypoint.pose.orientation.z;
        waypoint_map[name].quaternion.w() = msg->waypoint.pose.orientation.w;

        for (const auto &[key, value] : msg->waypoint.properties) {
            waypoint_map[name].properties[key] = value;
        }
        waypoint_map.setQuaternion(name);
        publishLatchedData();
    }

    void Node::appendRoute(const std_msgs::msg::String::SharedPtr msg) {
        router.append(msg->data);
        publishLatchedData();
    }

    void Node::eraseRoute(const std_msgs::msg::String::SharedPtr msg) {
        router.erase(msg->data);
        publishLatchedData();
    }

    void Node::insertRoute(const std_msgs::msg::String::SharedPtr msg) {
        if (!waypoint_map.hasKey(msg->data)) {
            RCLCPP_WARN(this->get_logger(), "Do not have waypoint name");
            return;
        }
        const auto name = generateKey(regist_goal_id, param.regist_waypoint_prefix);
        waypoint_map[name].goal = waypoint_map[msg->data].goal;
        waypoint_map[name].goal.x() += 1.0;
        waypoint_map.setQuaternion(name);

        if (router.insertFromKey(msg->data, name, true)) {
            RCLCPP_INFO(this->get_logger(), "Inserted route %s", name.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Failed insert %s", name.c_str());
        }
        publishLatchedData();
    }

    bool Node::save(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "Called save()");
        waypoint_map.save(param.waypoints_file);
        router.save(param.route_file);
        return true;
    }

    bool Node::saveWaypoints(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "Called saveWaypoints()");
        waypoint_map.save(param.waypoints_file);
        return true;
    }

    bool Node::saveRoute(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "Called saveRoute()");
        router.save(param.route_file);
        return true;
    }

    bool Node::resetRoute(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "Called resetRoute()");

        if (router.isEmpty()) {
            RCLCPP_WARN(this->get_logger(), "Route size of zero");
            return false;
        }
        router.resetIndex();
        publishGoal();

        RCLCPP_INFO(this->get_logger(), "Reset of route current goal %s", router.getIndex().c_str());

        return true;
    }

    bool Node::switchCancel(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "Called switchCancel()");
        exchangeCancelState();
        return true;
    }

    bool Node::nextWaypoint(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "Called nextWaypoint()");

        if (!router.forwardIndex()) {
            RCLCPP_WARN(this->get_logger(), "Failed forward index for route");
        }
        publishGoal();

        RCLCPP_WARN(this->get_logger(), "Call ClearCostmaps");
        RCLCPP_WARN(this->get_logger(), "%s", param.clear_costmap_srv.c_str());

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        clear_costmap_service->async_send_request(request);

        return true;
    }

    bool Node::prevWaypoint(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "Called prevWaypoint()");

        if (!router.backIndex()) {
            RCLCPP_WARN(this->get_logger(), "Failed back index for route");
        }
        publishGoal();

        return true;
    }

    void Node::publishGoal() {
        if (is_cancel.load()) {
            return;
        }
        if (router.isEmpty()) {
            return;
        }
        const auto pose_vector = waypoint_map[router.getIndex()].goal;
        const auto orientation = waypoint_map[router.getIndex()].quaternion;

        waypoint_manager_msgs::msg::Waypoint waypoint;

        waypoint.identity = router.getIndex();
        waypoint.pose.position.x = pose_vector.x();
        waypoint.pose.position.y = pose_vector.y();
        waypoint.pose.position.z = pose_vector.z();
        waypoint.pose.orientation.x = orientation.x();
        waypoint.pose.orientation.y = orientation.y();
        waypoint.pose.orientation.z = orientation.z();
        waypoint.pose.orientation.w = orientation.w();

        for (const auto &[name, value] : waypoint_map[router.getIndex()].properties) {
            waypoint_manager_msgs::msg::Property property;

            property.name = name;
            property.data = value;

            waypoint.properties.push_back(property);
        }

        waypoint_publisher->publish(waypoint);
    }

    void Node::publishWaypoints() {
        waypoint_manager_msgs::msg::Waypoints waypoints_msg;

        for (const auto &[key, waypoint] : waypoint_map.data()) {
            waypoint_manager_msgs::msg::Waypoint waypoint_msg;

            waypoint_msg.identity = key;
            waypoint_msg.pose.position.x = waypoint.goal.x();
            waypoint_msg.pose.position.y = waypoint.goal.y();
            waypoint_msg.pose.position.z = waypoint.goal.z();
            waypoint_msg.pose.orientation.x = waypoint.quaternion.x();
            waypoint_msg.pose.orientation.y = waypoint.quaternion.y();
            waypoint_msg.pose.orientation.z = waypoint.quaternion.z();
            waypoint_msg.pose.orientation.w = waypoint.quaternion.w();

            for (const auto &[name, data] : waypoint.properties) {
                waypoint_manager_msgs::msg::Property property;

                property.name = name;
                property.data = data;

                waypoint_msg.properties.push_back(property);
            }
            waypoints_msg.waypoints.push_back(waypoint_msg);
        }
        waypoints_msg.info.header.frame_id = param.global_frame;
        waypoints_msg.info.header.stamp = this->now();

        waypoints_publisher->publish(waypoints_msg);
    }

    void Node::publishRoute() {
        waypoint_manager_msgs::msg::Route route_msg;

        for (const auto &id : router.data()) {
            route_msg.identities.push_back(id);
        }
        route_msg.header.frame_id = param.global_frame;
        route_msg.header.stamp = this->now();

        route_publisher->publish(route_msg);
    }

    void Node::publishLatchedData() {
        publishRoute();
        rclcpp::sleep_for(std::chrono::duration<double>(param.wait_publish_waypoints_time));
        publishWaypoints();
    }

    void Node::exchangeCancelState() {
        is_cancel.store(!is_cancel.load());

        if (is_cancel.load()) {
            RCLCPP_INFO(this->get_logger(), "Changed is_cancel true");
        } else {
            RCLCPP_INFO(this->get_logger(), "Changed is_cancel false");
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("waypoint_server_node"), "Start waypoint_server_node");

    auto node = std::make_shared<waypoint_server::Node>();
    node->spin();

    RCLCPP_INFO(rclcpp::get_logger("waypoint_server_node"), "Finish waypoint_server_node");
    rclcpp::shutdown();
    return 0;
}
