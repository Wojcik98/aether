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

#include <aether/dijkstra_path_finder.hpp>
#include <aether/map_interface.hpp>
#include <aether/mapped_localization.hpp>
#include <aether/robot_config.hpp>

#include "aether_app/online_file_map.hpp"

using namespace std::chrono_literals;

class PathVisualizer : public rclcpp::Node {
public:
    PathVisualizer() : Node("path_visualizer") {
        this->declare_parameter("map_path", "");
        map_path_ = this->get_parameter("map_path").as_string();

        if (map_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "Map path is empty");
            throw std::runtime_error("Map path is empty");
        }
        RCLCPP_INFO(get_logger(), "Map path: %s", map_path_.c_str());

        map_ = std::make_shared<OnlineFileMap>(map_path_);
        map_confident_ = std::make_shared<MapConfident<OnlineFileMap>>(*map_);
        path_finder_ = std::make_shared<DijkstraPathFinder<OnlineFileMap>>(
            *map_confident_);
        planner_ =
            std::make_shared<SimplePlanner>(path_finder_->path, 0.1f, 0.5f);

        CellCoords start = {0, -1};
        CellCoords goal = {
            MAZE_SIZE_X_CELLS / 2,
            -(int32_t)MAZE_SIZE_Y_CELLS / 2,
        };
        path_finder_->find_path(start, goal);

        // set transient QoS
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.transient_local();
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", qos);

        publish_path();
        RCLCPP_INFO(this->get_logger(), "Path published");
    }

    void publish_path() {
        constexpr uint32_t SKIP_POINTS = FREQ_CONTROL / 5;
        FullState state = {0.09f, -0.09f, STARTING_YAW, 0.0f, 0.0f};
        planner_->start(state);

        auto msg = nav_msgs::msg::Path();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        tf2::Quaternion q;

        while (!planner_->goal_reached()) {
            auto state = planner_->get_next_state();
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
                planner_->get_next_state();
            }
        }

        path_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Path published");
    }

private:
    std::string map_path_;
    std::shared_ptr<OnlineFileMap> map_;
    std::shared_ptr<MapConfident<OnlineFileMap>> map_confident_;
    std::shared_ptr<MappedLocalization<OnlineFileMap>> localization_;
    std::shared_ptr<DijkstraPathFinder<OnlineFileMap>> path_finder_;
    std::shared_ptr<SimplePlanner> planner_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathVisualizer>());
    rclcpp::shutdown();
    return 0;
}
