#include <array>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <aether_msgs/msg/imu_enc_synced.hpp>
#include <aether_msgs/msg/tofs_synced.hpp>

#include <aether/controller.hpp>
#include <aether/dijkstra_path_finder.hpp>
#include <aether/map_interface.hpp>
#include <aether/mapped_localization.hpp>
#include <aether/robot_config.hpp>
#include <aether/simple_planner.hpp>

#include "aether_app/online_file_map.hpp"

using namespace std::chrono_literals;

class ExecutePath : public rclcpp::Node {
public:
    ExecutePath(const rclcpp::NodeOptions &options)
        : Node("execute_path", options) {
        this->declare_parameter("map_path", "");
        this->declare_parameter("kp_x", 1.0f);
        this->declare_parameter("kp_y", 1.0f);
        this->declare_parameter("kp_om", 1.0f);
        this->declare_parameter("vx", 0.1f);
        this->declare_parameter("om", 0.5f);
        map_path_ = this->get_parameter("map_path").as_string();
        float kp_x = this->get_parameter("kp_x").as_double();
        float kp_y = this->get_parameter("kp_y").as_double();
        float kp_om = this->get_parameter("kp_om").as_double();
        float vx = this->get_parameter("vx").as_double();
        float om = this->get_parameter("om").as_double();

        if (map_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "Map path is empty");
            throw std::runtime_error("Map path is empty");
        }
        RCLCPP_INFO(get_logger(), "Map path: %s", map_path_.c_str());

        map_ = std::make_shared<OnlineFileMap>(map_path_);
        map_confident_ = std::make_shared<MapConfident<OnlineFileMap>>(*map_);
        localization_ = std::make_shared<MappedLocalization<OnlineFileMap>>(
            *map_confident_);
        path_finder_ = std::make_shared<DijkstraPathFinder<OnlineFileMap>>(
            *map_confident_);
        planner_ = std::make_shared<SimplePlanner>(path_finder_->path, vx, om);
        RCLCPP_INFO(get_logger(), "Map loaded");

        controller_.set_KPx(kp_x);
        controller_.set_KPy(kp_y);
        controller_.set_KPom(kp_om);

        CellCoords start = {0, -1};
        CellCoords goal = {
            MAZE_SIZE_X_CELLS / 2,
            -(int32_t)MAZE_SIZE_Y_CELLS / 2,
        };
        path_finder_->find_path(start, goal);

        imu_enc_sub_ = this->create_subscription<ImuEncSynced>(
            "imu_enc", 10,
            std::bind(&ExecutePath::imu_enc_callback, this,
                      std::placeholders::_1));
        tofs_sub_ = this->create_subscription<TofsSynced>(
            "tofs", 10,
            std::bind(&ExecutePath::tofs_callback, this,
                      std::placeholders::_1));

        cmd_vel_pub_ =
            this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        constexpr float control_dt = 1.0f / FREQ_CONTROL;
        timer_ = this->create_wall_timer(
            std::chrono::duration<float>(control_dt),
            std::bind(&ExecutePath::control_callback, this));

        start_srv_ = this->create_service<std_srvs::srv::Empty>(
            "start", std::bind(&ExecutePath::start_navigation, this,
                               std::placeholders::_1, std::placeholders::_2));
    }

private:
    std::string map_path_;
    std::shared_ptr<OnlineFileMap> map_;
    std::shared_ptr<MapConfident<OnlineFileMap>> map_confident_;
    std::shared_ptr<MappedLocalization<OnlineFileMap>> localization_;
    std::shared_ptr<DijkstraPathFinder<OnlineFileMap>> path_finder_;
    std::shared_ptr<SimplePlanner> planner_;
    Controller controller_;
    bool start_navigation_ = false;

    using Imu = sensor_msgs::msg::Imu;
    using Range = sensor_msgs::msg::Range;
    using Odometry = nav_msgs::msg::Odometry;
    using Marker = visualization_msgs::msg::Marker;
    using ImuEncSynced = aether_msgs::msg::ImuEncSynced;
    using TofsSynced = aether_msgs::msg::TofsSynced;

    rclcpp::Subscription<ImuEncSynced>::SharedPtr imu_enc_sub_;
    rclcpp::Subscription<TofsSynced>::SharedPtr tofs_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv_;

    void start_navigation(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                          std::shared_ptr<std_srvs::srv::Empty::Response>) {
        auto state = localization_->get_latest_pose();
        planner_->start(state);
        start_navigation_ = true;
    }

    void imu_enc_callback(const ImuEncSynced::ConstSharedPtr &msg) {
        auto &imu_msg = msg->imu;
        auto &odometry_msg = msg->enc;

        ImuData imu_data(imu_msg.angular_velocity.z);

        float odom_vel_lin = odometry_msg.twist.twist.linear.x;
        float odom_vel_ang = odometry_msg.twist.twist.angular.z;
        // angular velocities of each wheel
        float om_l =
            (odom_vel_lin - odom_vel_ang * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
        float om_r =
            (odom_vel_lin + odom_vel_ang * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
        EncoderData encoder_data(om_l, om_r);

        localization_->imu_enc_update(imu_data, encoder_data);
    }

    void tofs_callback(const TofsSynced::ConstSharedPtr &msg) {
        const float std = 0.05f;
        TofsReadings tofs_data = {
            {tof_infinitize(msg->tofs[0].range), std},
            {tof_infinitize(msg->tofs[1].range), std},
            {tof_infinitize(msg->tofs[2].range), std},
            {tof_infinitize(msg->tofs[3].range), std},
            {tof_infinitize(msg->tofs[4].range), std},
            {tof_infinitize(msg->tofs[5].range), std},
        };

        RCLCPP_DEBUG(get_logger(), "TOF readings: %f, %f, %f, %f, %f, %f",
                     tofs_data[0].dist, tofs_data[1].dist, tofs_data[2].dist,
                     tofs_data[3].dist, tofs_data[4].dist, tofs_data[5].dist);
        localization_->tofs_update(tofs_data);
        auto pose = localization_->get_latest_pose();
        if (std::isnan(pose.x) || std::isnan(pose.y) || std::isnan(pose.yaw)) {
            RCLCPP_ERROR(get_logger(), "NaN pose");
            throw std::runtime_error("NaN pose");
        }
    }

    float tof_infinitize(float dist) {
        return dist > TOF_RANGE ? INFINITY : dist;
    }

    void control_callback() {
        tf2::Quaternion q;
        auto pose = localization_->get_latest_pose();
        if (std::isnan(pose.x) || std::isnan(pose.y) || std::isnan(pose.yaw)) {
            RCLCPP_ERROR(get_logger(), "NaN pose");
            throw std::runtime_error("NaN pose");
        }
        RCLCPP_DEBUG(get_logger(), "Pose: (%f, %f, %f)", pose.x, pose.y,
                     pose.yaw);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = pose.x;
        transform.transform.translation.y = pose.y;
        transform.transform.translation.z = 0.0;
        q.setRPY(0, 0, pose.yaw);
        transform.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(transform);

        if (!start_navigation_) {
            return;
        }

        auto target = planner_->get_next_state();
        auto target_local = pose.to_pose().inverse() * target.to_pose();
        geometry_msgs::msg::TransformStamped target_transform;
        target_transform.header.stamp = this->now();
        target_transform.header.frame_id = "base_link";
        target_transform.child_frame_id = "target";
        target_transform.transform.translation.x = target_local.x;
        target_transform.transform.translation.y = target_local.y;
        target_transform.transform.translation.z = 0.0;
        q.setRPY(0, 0, target_local.yaw);
        target_transform.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(target_transform);

        auto twist = controller_.get_twist(pose, target);
        if (planner_->goal_reached()) {
            RCLCPP_INFO(get_logger(), "Goal reached");
            start_navigation_ = false;
            twist.vx = 0.0f;
            twist.omega = 0.0f;
        }

        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = twist.vx;
        twist_msg.angular.z = twist.omega;
        cmd_vel_pub_->publish(twist_msg);
    }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ExecutePath)
