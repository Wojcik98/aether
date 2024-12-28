#include <functional>
#include <memory>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <aether_msgs/msg/imu_enc_synced.hpp>
#include <aether_msgs/msg/tofs_synced.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;
using std::placeholders::_6;

class SensorsSync : public rclcpp::Node {
public:
    SensorsSync(const rclcpp::NodeOptions &options)
        : Node("sensors_sync", options) {
        tofs_pub_ =
            this->create_publisher<aether_msgs::msg::TofsSynced>("tofs", 10);
        imu_enc_pub_ = this->create_publisher<aether_msgs::msg::ImuEncSynced>(
            "imu_enc", 10);

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
            std::bind(&SensorsSync::imu_enc_callback, this, _1, _2));

        tofs_sync_ = std::make_shared<TofsSync>(tof_subs_[0], tof_subs_[1],
                                                tof_subs_[2], tof_subs_[3],
                                                tof_subs_[4], tof_subs_[5], 10);
        tofs_sync_->registerCallback(std::bind(&SensorsSync::tofs_callback,
                                               this, _1, _2, _3, _4, _5, _6));
    }

private:
    // don't want to have dependency on aether
    static constexpr int8_t NUM_TOFS = 6;

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

    std::shared_ptr<rclcpp::Publisher<aether_msgs::msg::TofsSynced>> tofs_pub_;
    std::shared_ptr<rclcpp::Publisher<aether_msgs::msg::ImuEncSynced>>
        imu_enc_pub_;

    void imu_enc_callback(const Imu::ConstSharedPtr &imu_msg,
                          const Odometry::ConstSharedPtr &odometry_msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received IMU and odometry messages");
        aether_msgs::msg::ImuEncSynced msg;
        msg.imu = *imu_msg;
        msg.enc = *odometry_msg;
        imu_enc_pub_->publish(msg);
    }

    void tofs_callback(const Range::ConstSharedPtr &tof_right_side_msg,
                       const Range::ConstSharedPtr &tof_right_diag_msg,
                       const Range::ConstSharedPtr &tof_right_front_msg,
                       const Range::ConstSharedPtr &tof_left_front_msg,
                       const Range::ConstSharedPtr &tof_left_diag_msg,
                       const Range::ConstSharedPtr &tof_left_side_msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received TOF messages");
        aether_msgs::msg::TofsSynced msg;
        msg.tofs.resize(NUM_TOFS);
        msg.tofs[0] = *tof_right_side_msg;
        msg.tofs[1] = *tof_right_diag_msg;
        msg.tofs[2] = *tof_right_front_msg;
        msg.tofs[3] = *tof_left_front_msg;
        msg.tofs[4] = *tof_left_diag_msg;
        msg.tofs[5] = *tof_left_side_msg;
        tofs_pub_->publish(msg);
    }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SensorsSync)
