#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>

using namespace std::chrono_literals;
using Topics = std::vector<std::string>;
using LaserScan = sensor_msgs::msg::LaserScan;
using Range = sensor_msgs::msg::Range;

class LidarToRange : public rclcpp::Node {
public:
    LidarToRange() : Node("lidar_to_range_node") {
        this->declare_parameter<Topics>("lidar_topics", Topics{});
        this->declare_parameter<Topics>("range_topics", Topics{});
        this->declare_parameter<double>("fov", 0.314159); // 18 degrees

        auto lidar_topics =
            this->get_parameter("lidar_topics").as_string_array();
        auto range_topics =
            this->get_parameter("range_topics").as_string_array();
        float fov = this->get_parameter("fov").as_double();

        if (lidar_topics.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No lidar topics specified");
            return;
        }
        if (range_topics.empty()) {
            for (const auto &lidar_topic : lidar_topics) {
                auto range_topic = lidar_topic + "_range";
                range_topics.push_back(range_topic);
            }
        }
        if (lidar_topics.size() != range_topics.size()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Number of lidar topics and range topics do not match");
            return;
        }

        lidar_subs_.reserve(lidar_topics.size());
        range_pubs_.reserve(range_topics.size());

        for (size_t i = 0; i < lidar_topics.size(); i++) {
            auto range_pub = this->create_publisher<Range>(range_topics[i], 10);
            range_pubs_.push_back(range_pub);

            auto callback = [this, i, fov](const LaserScan::SharedPtr msg) {
                if (msg->ranges.size() != 1) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Expected exactly one range value, but got %ld",
                        msg->ranges.size());
                    return;
                }
                auto range_msg = std::make_shared<Range>();
                range_msg->header = msg->header;
                range_msg->radiation_type = Range::INFRARED;
                range_msg->field_of_view = fov;
                range_msg->min_range = msg->range_min;
                range_msg->max_range = msg->range_max;
                range_msg->range = msg->ranges[0];
                this->range_pubs_[i]->publish(*range_msg);
            };
            auto lidar_sub = this->create_subscription<LaserScan>(
                lidar_topics[i], 10, callback);
            lidar_subs_.push_back(lidar_sub);
        }
    }

private:
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr>
        lidar_subs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr>
        range_pubs_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarToRange>());
    rclcpp::shutdown();
    return 0;
}
