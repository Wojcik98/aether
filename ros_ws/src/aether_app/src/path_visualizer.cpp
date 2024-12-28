#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <aether/robot_config.hpp>
#include <aether/simple_planner.hpp>
#include <aether/types.hpp>

#include "aether_app/file_path_finder.hpp"

using namespace std::chrono_literals;

class PathVisualizer : public rclcpp::Node {
public:
    PathVisualizer() : Node("path_visualizer") {
        this->declare_parameter("path_path", "");
        std::string path_path = this->get_parameter("path_path").as_string();
        path_finder_.load_file(path_path);

        // set transient QoS
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.transient_local();
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", qos);

        publish_path();
        RCLCPP_INFO(this->get_logger(), "Path published");
    }

    void publish_path() {
        constexpr uint32_t NUM_POINTS = 150;
        constexpr uint32_t SKIP_POINTS = 20;
        FullState state = {0.09f, -0.09f, STARTING_YAW, 0.0f, 0.0f};
        planner_.start(state);

        auto msg = nav_msgs::msg::Path();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        tf2::Quaternion q;

        for (uint32_t i = 0; i < NUM_POINTS; i++) {
            auto state = planner_.get_next_state();
            auto pose = geometry_msgs::msg::PoseStamped();
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = state.x;
            pose.pose.position.y = state.y;
            pose.pose.position.z = 0.0;
            q.setRPY(0, 0, state.yaw);
            pose.pose.orientation = tf2::toMsg(q);
            msg.poses.push_back(pose);

            for (uint32_t j = 0; j < SKIP_POINTS; j++) {
                planner_.get_next_state();
            }
            if (planner_.goal_reached()) {
                break;
            }
        }

        path_pub_->publish(msg);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    FilePathFinder path_finder_;
    SimplePlanner planner_{path_finder_.path, 0.1f, 0.5f};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathVisualizer>());
    rclcpp::shutdown();
    return 0;
}
