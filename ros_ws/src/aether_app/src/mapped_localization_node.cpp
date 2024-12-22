#include <array>
#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <aether/map_interface.hpp>
#include <aether/mapped_localization.hpp>
#include <aether/robot_config.hpp>

#include "aether_app/online_file_map.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;
using std::placeholders::_6;

class MappedLocalizationNode : public rclcpp::Node {
public:
    MappedLocalizationNode() : Node("mapped_localization_node") {
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

        // TODO: message filters seem to take a long time to compile
        // I could do synchronization in a separate node and use composition
        imu_sub_.subscribe(this, "imu");
        odometry_sub_.subscribe(this, "odom");
        tof_subs_[0].subscribe(this, "tof_right_side");
        tof_subs_[1].subscribe(this, "tof_right_diag");
        tof_subs_[2].subscribe(this, "tof_right_front");
        tof_subs_[3].subscribe(this, "tof_left_front");
        tof_subs_[4].subscribe(this, "tof_left_diag");
        tof_subs_[5].subscribe(this, "tof_left_side");

        imu_enc_sync_ =
            std::make_shared<ImuEncSync>(imu_sub_, odometry_sub_, 10);
        imu_enc_sync_->registerCallback(
            std::bind(&MappedLocalizationNode::imu_enc_callback, this, _1, _2));

        tofs_sync_ = std::make_shared<TofsSync>(tof_subs_[0], tof_subs_[1],
                                                tof_subs_[2], tof_subs_[3],
                                                tof_subs_[4], tof_subs_[5], 10);
        tofs_sync_->registerCallback(
            std::bind(&MappedLocalizationNode::tofs_callback, this, _1, _2, _3,
                      _4, _5, _6));

        // odometry_pub_ = this->create_publisher<Odometry>("odom_out", 10);
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&MappedLocalizationNode::timer_callback, this));
    }

private:
    std::string map_path_;
    std::shared_ptr<OnlineFileMap> map_;
    std::shared_ptr<MapConfident<OnlineFileMap>> map_confident_;
    std::shared_ptr<MappedLocalization<OnlineFileMap>> localization_;

    using Imu = sensor_msgs::msg::Imu;
    using Range = sensor_msgs::msg::Range;
    using Odometry = nav_msgs::msg::Odometry;

    using ImuEncSync = message_filters::TimeSynchronizer<Imu, Odometry>;
    using TofsSync = message_filters::TimeSynchronizer<Range, Range, Range,
                                                       Range, Range, Range>;

    message_filters::Subscriber<Imu> imu_sub_;
    message_filters::Subscriber<Odometry> odometry_sub_;
    std::array<message_filters::Subscriber<Range>, NUM_TOFS> tof_subs_;

    std::shared_ptr<ImuEncSync> imu_enc_sync_;
    std::shared_ptr<TofsSync> tofs_sync_;

    // rclcpp::Publisher<Odometry>::SharedPtr odometry_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    void imu_enc_callback(const Imu::ConstSharedPtr &imu_msg,
                          const Odometry::ConstSharedPtr &odometry_msg) {
        ImuData imu_data(imu_msg->angular_velocity.z);

        float odom_vel_lin = odometry_msg->twist.twist.linear.x;
        float odom_vel_ang = odometry_msg->twist.twist.angular.z;
        // angular velocities of each wheel
        float om_l =
            (odom_vel_lin - odom_vel_ang * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
        float om_r =
            (odom_vel_lin + odom_vel_ang * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
        EncoderData encoder_data(om_l, om_r);

        localization_->imu_enc_update(0, imu_data, encoder_data);
    }

    void tofs_callback(const Range::ConstSharedPtr &tof_right_side_msg,
                       const Range::ConstSharedPtr &tof_right_diag_msg,
                       const Range::ConstSharedPtr &tof_right_front_msg,
                       const Range::ConstSharedPtr &tof_left_front_msg,
                       const Range::ConstSharedPtr &tof_left_diag_msg,
                       const Range::ConstSharedPtr &tof_left_side_msg) {
        const float std = 0.05f;
        TofsReadings tofs_data = {
            {tof_infinitize(tof_right_side_msg->range), std},
            {tof_infinitize(tof_right_diag_msg->range), std},
            {tof_infinitize(tof_right_front_msg->range), std},
            {tof_infinitize(tof_left_front_msg->range), std},
            {tof_infinitize(tof_left_diag_msg->range), std},
            {tof_infinitize(tof_left_side_msg->range), std},
        };

        RCLCPP_DEBUG(get_logger(), "TOF readings: %f, %f, %f, %f, %f, %f",
                     tofs_data[0].dist, tofs_data[1].dist, tofs_data[2].dist,
                     tofs_data[3].dist, tofs_data[4].dist, tofs_data[5].dist);
        localization_->tofs_update(0, tofs_data);
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
        RCLCPP_INFO(get_logger(), "Pose: (%f, %f, %f)", pose.x, pose.y,
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

        // Odometry odom_msg;
        // odom_msg.header.stamp = this->now();
        // odom_msg.header.frame_id = "map";
        // odom_msg.child_frame_id = "base_link";
        // odom_msg.pose.pose.position.x = pose.x;
        // odom_msg.pose.pose.position.y = pose.y;
        // odom_msg.pose.pose.position.z = 0.0;
        // odom_msg.pose.pose.orientation.x = 0.0;
        // odom_msg.pose.pose.orientation.y = 0.0;
        // odom_msg.pose.pose.orientation.z = sin(pose.yaw / 2.0);
        // odom_msg.pose.pose.orientation.w = cos(pose.yaw / 2.0);
        // odometry_pub_->publish(odom_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MappedLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
