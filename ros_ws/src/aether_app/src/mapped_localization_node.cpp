#include <array>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <aether_msgs/msg/imu_enc_synced.hpp>
#include <aether_msgs/msg/tofs_synced.hpp>

#include <aether/map_interface.hpp>
#include <aether/mapped_localization.hpp>
#include <aether/robot_config.hpp>

#include "aether_app/online_file_map.hpp"

using namespace std::chrono_literals;

class MappedLocalizationNode : public rclcpp::Node {
public:
    MappedLocalizationNode(const rclcpp::NodeOptions &options)
        : Node("mapped_localization_node", options) {
        this->declare_parameter("map_path", "");
        map_path_ = this->get_parameter("map_path").as_string();

        if (map_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "Map path is empty");
            throw std::runtime_error("Map path is empty");
        }
        RCLCPP_INFO(get_logger(), "Map path: %s", map_path_.c_str());

        map_ = std::make_shared<OnlineFileMap>(map_path_);
        map_confident_ = std::make_shared<MapConfident<OnlineFileMap>>(*map_);
        localization_ = std::make_shared<MappedLocalization<OnlineFileMap>>(
            *map_confident_);
        RCLCPP_INFO(get_logger(), "Map loaded");

        imu_enc_sub_ = this->create_subscription<ImuEncSynced>(
            "imu_enc", 10,
            std::bind(&MappedLocalizationNode::imu_enc_callback, this,
                      std::placeholders::_1));
        tofs_sub_ = this->create_subscription<TofsSynced>(
            "tofs", 10,
            std::bind(&MappedLocalizationNode::tofs_callback, this,
                      std::placeholders::_1));

        // odometry_pub_ = this->create_publisher<Odometry>("odom_out", 10);
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        particles_pub_ = this->create_publisher<Marker>("particles", 10);
        timer_ = this->create_wall_timer(
            50ms, std::bind(&MappedLocalizationNode::timer_callback, this));
    }

private:
    std::string map_path_;
    std::shared_ptr<OnlineFileMap> map_;
    std::shared_ptr<MapConfident<OnlineFileMap>> map_confident_;
    std::shared_ptr<MappedLocalization<OnlineFileMap>> localization_;

    using Imu = sensor_msgs::msg::Imu;
    using Range = sensor_msgs::msg::Range;
    using Odometry = nav_msgs::msg::Odometry;
    using Marker = visualization_msgs::msg::Marker;
    using ImuEncSynced = aether_msgs::msg::ImuEncSynced;
    using TofsSynced = aether_msgs::msg::TofsSynced;

    rclcpp::Subscription<ImuEncSynced>::SharedPtr imu_enc_sub_;
    rclcpp::Subscription<TofsSynced>::SharedPtr tofs_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<rclcpp::Publisher<Marker>> particles_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

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

    void timer_callback() {
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
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = sin(pose.yaw / 2.0);
        transform.transform.rotation.w = cos(pose.yaw / 2.0);
        tf_broadcaster_->sendTransform(transform);

        auto particles = localization_->get_particles();
        Marker msg;
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        msg.ns = "particles";
        msg.id = 0;
        msg.type = Marker::SPHERE_LIST;
        msg.action = Marker::ADD;
        msg.pose.orientation.w = 1.0;
        msg.scale.x = 0.01;
        msg.scale.y = 0.01;
        msg.color.r = 1.0;
        msg.color.a = 1.0;
        for (const auto &particle : particles) {
            geometry_msgs::msg::Point p;
            p.x = particle.state.x;
            p.y = particle.state.y;
            p.z = 0.0;
            msg.points.push_back(p);
        }
        particles_pub_->publish(msg);
    }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MappedLocalizationNode)
