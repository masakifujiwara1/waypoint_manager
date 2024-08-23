
#include <limits>
#include <atomic>
#include <string>
#include <stdexcept>
#include <exception>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <time.h>

#include <std_msgs/Bool.h>
#include <waypoint_manager_msgs/Waypoint.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace {
    static std::atomic_bool recived_waypoint, stop_waypoint;
    static float default_goal_radius = 1;
    static float current_goal_radius = default_goal_radius;
    static Eigen::Vector2f current_position = Eigen::Vector2f::Zero();
    static Eigen::Vector2f old_current_position = Eigen::Vector2f::Zero();
    static time_t start_time, last_moving_time;
    static std::atomic_bool is_fst_flag, is_fst_waypoint_reached, is_reached_goal, is_to_prev_waypoint;
    static std::string old_id;
    static double delta_pose_dist = 0, pose_dist = 0;
    static float vel_x = 0;
    static float limit_delta_pose_dist = 0.1;
    static float limit_time = 20;
}

void waypointCallback(const waypoint_manager_msgs::Waypoint::ConstPtr &msg) {
    try {
        // first callback process
        if (is_fst_flag) {
            old_id = msg->identity;
            is_fst_flag.store(false);
            start_time = time(NULL);
            return;
        }

        // check switch waypoint
        if (msg->identity != old_id) {
            start_time = time(NULL);
            is_fst_waypoint_reached.store(true);
        }

        // ROS_WARN("recived_waypoint");        
        recived_waypoint.store(true);

        old_id = msg->identity;
    }
    catch(const std::exception &) {
        recived_waypoint.store(false);
        ROS_WARN("Failed parse check_robot_moving_node");
    }
}

void MclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    try {
        // ROS_WARN("recived_mcl_pose");        
        current_position.x() = msg->pose.pose.position.x;
        current_position.y() = msg->pose.pose.position.y;

        if(recived_waypoint.load()) {
            delta_pose_dist = std::sqrt(std::pow(current_position.x() - old_current_position.x(), 2) + std::pow(current_position.y() - old_current_position.y(), 2)) * 5.0;
        }

        old_current_position = current_position;
    }
    catch(const std::exception &) {
        // recived_waypoint.store(false);
        ROS_WARN("Failed mcl_pose");
    }
}

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    try {
        // is_reached_goal.store(msg->data); 
        vel_x = msg->linear.x;      
    }
    catch(const std::exception &) {
        // recived_waypoint.store(false);
        ROS_WARN("Failed cmd_vel");
    }
}

void IsReachedGoalCallback(const std_msgs::Bool::ConstPtr &msg) {
    try {
        is_reached_goal.store(msg->data);        
    }
    catch(const std::exception &) {
        // recived_waypoint.store(false);
        ROS_WARN("Failed is_reached_goal");
    }
}

auto main(int argc, char **argv) -> int {
    ros::init(argc, argv, "check_robot_moving_node");
    ros::NodeHandle nh,
                    private_nh("~");

    std::string goal_topic,
                waypoint_topic,
                is_reached_goal_topic,
                mcl_pose_topic,
                cmd_vel_topic,
                clear_costmap_srv;

    std::string robot_base_frame,
                global_frame;

    float goal_check_frequency,
          wait_no_waypoint_time;

    recived_waypoint.store(false);
    stop_waypoint.store(false);
    is_fst_flag.store(true);
    is_fst_waypoint_reached.store(false);
    is_reached_goal.store(false);
    is_to_prev_waypoint.store(false);

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
    private_nh.param(
        "mcl_pose_topic",
        mcl_pose_topic,
        std::string("mcl_pose")
    );
    private_nh.param(
        "cmd_vel_topic",
        cmd_vel_topic,
        std::string("icart_mini/cmd_vel")
    );
    private_nh.param(
        "clear_costmap_srv",
        clear_costmap_srv,
        std::string("move_base/clear_costmaps")
    );
    private_nh.param(
        "limit_time",
        limit_time,
        static_cast<float>(20.0)
    );
    private_nh.param(
        "limit_delta_pose_dist",
        limit_delta_pose_dist,
        static_cast<float>(0.1)
    );

    auto loop_rate = ros::Rate(5);
    auto waypoint_subscriber = nh.subscribe(
        waypoint_topic,
        1,
        waypointCallback
    );
    auto mcl_pose_subscriber = nh.subscribe(
        mcl_pose_topic,
        2,
        MclPoseCallback
    );
    auto cmd_vel_subscriber = nh.subscribe(
        cmd_vel_topic,
        2,
        CmdVelCallback
    );
    auto is_reached_goal_subscriber = nh.subscribe<std_msgs::Bool>(
        is_reached_goal_topic,
        1,
        IsReachedGoalCallback
    );
    auto prev_waypoint_service = nh.serviceClient<std_srvs::Trigger>("waypoint_server/prev_waypoint");
    auto clear_costmap_service = private_nh.serviceClient<std_srvs::Empty>(clear_costmap_srv);

    ROS_INFO("Start check_robot_moving_node");

    while(ros::ok()) {
        ros::spinOnce();

        if (!recived_waypoint.load()) {
            ROS_INFO("Waiting waypoint check_moving_node");
            ros::Duration(1.0).sleep();
            last_moving_time = time(NULL);
            continue;
        }

        // moving toward previous waypoint
        if (is_reached_goal && is_to_prev_waypoint) {
            std_srvs::Empty data;
            clear_costmap_service.call(data);
            is_to_prev_waypoint.store(false);
            ROS_WARN("Clear Costmaps");
        }

        // check robot delta pose dist
        if (delta_pose_dist <= limit_delta_pose_dist && !is_reached_goal) {
            ROS_INFO("time:%ld, stopped time:%ld\n", time(NULL) - start_time, time(NULL) - last_moving_time);

            if (time(NULL) - last_moving_time >= limit_time) {
                std_srvs::Trigger trigger;
                if (is_fst_waypoint_reached) {
                    ROS_INFO("Service call PrevWaypoint()");
                    prev_waypoint_service.call(trigger);
                    is_to_prev_waypoint.store(true);
                    last_moving_time = time(NULL);
                }
            }
        } else {
            last_moving_time = time(NULL);
        }

        loop_rate.sleep();
    }
    ROS_INFO("Finish waypoint_server_node");

    return 0;
}

