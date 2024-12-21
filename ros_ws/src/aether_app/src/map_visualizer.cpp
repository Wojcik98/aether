#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <aether/map_interface.hpp>
#include <aether/mapped_localization.hpp>
#include <aether/robot_config.hpp>

#include "aether_app/online_file_map.hpp"

using namespace std::chrono_literals;

class MapVisualizer : public rclcpp::Node {
public:
    MapVisualizer() : Node("map_visualizer") {
        this->declare_parameter("map_path", "");
        map_path_ = this->get_parameter("map_path").as_string();

        if (map_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "Map path is empty");
            throw std::runtime_error("Map path is empty");
        }
        RCLCPP_INFO(get_logger(), "Map path: %s", map_path_.c_str());

        map_ = std::make_shared<OnlineFileMap>(map_path_);
        RCLCPP_INFO(get_logger(), "Map loaded");

        // set Transient QoS
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.transient_local();
        maze_markers_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "maze_markers", qos);

        publish_maze_markers();
    }

    void publish_maze_markers() {
        constexpr float WALL_HEIGHT = 0.05;
        auto msg = visualization_msgs::msg::MarkerArray();
        int marker_id = 0;

        const auto &horizontal_walls = map_->get_horizontal_walls();
        const auto &vertical_walls = map_->get_vertical_walls();

        auto header = std_msgs::msg::Header();
        header.frame_id = "map";
        header.stamp = this->now();

        auto no_rotation = geometry_msgs::msg::Quaternion();
        no_rotation.x = 0.0;
        no_rotation.y = 0.0;
        no_rotation.z = 0.0;
        no_rotation.w = 1.0;

        auto rotation_90 = geometry_msgs::msg::Quaternion();
        rotation_90.x = 0.0;
        rotation_90.y = 0.0;
        rotation_90.z = -0.707;
        rotation_90.w = 0.707;

        auto wall_scale = geometry_msgs::msg::Vector3();
        wall_scale.x = CELL_INNER_SIZE;
        wall_scale.y = WALL_WIDTH;
        wall_scale.z = WALL_HEIGHT;

        auto wall_color = std_msgs::msg::ColorRGBA();
        wall_color.r = 0.9;
        wall_color.g = 0.9;
        wall_color.b = 0.9;
        wall_color.a = 1.0;

        auto post_color = std_msgs::msg::ColorRGBA();
        post_color.r = 0.8;
        post_color.g = 0.8;
        post_color.b = 0.8;
        post_color.a = 1.0;

        // horizontal walls
        for (size_t x = 0; x < horizontal_walls.size(); ++x) {
            for (size_t y = 0; y < horizontal_walls[x].size(); ++y) {
                if (horizontal_walls[x][y] > 0.5f) {
                    auto marker = visualization_msgs::msg::Marker();
                    marker.header = header;
                    marker.ns = "maze";
                    marker.id = marker_id++;
                    marker.type = visualization_msgs::msg::Marker::CUBE;
                    marker.action = visualization_msgs::msg::Marker::ADD;
                    marker.pose.position.x = x * CELL_SIZE;
                    marker.pose.position.y = -(y * CELL_SIZE) - CELL_SIZE / 2.0;
                    marker.pose.position.z = WALL_HEIGHT / 2.0;
                    marker.pose.orientation = rotation_90;
                    marker.scale = wall_scale;
                    marker.color = wall_color;
                    msg.markers.push_back(marker);
                }
            }
        }

        // vertical walls
        for (size_t x = 0; x < vertical_walls.size(); ++x) {
            for (size_t y = 0; y < vertical_walls[x].size(); ++y) {
                if (vertical_walls[x][y] > 0.5f) {
                    auto marker = visualization_msgs::msg::Marker();
                    marker.header = header;
                    marker.ns = "maze";
                    marker.id = marker_id++;
                    marker.type = visualization_msgs::msg::Marker::CUBE;
                    marker.action = visualization_msgs::msg::Marker::ADD;
                    marker.pose.position.x = x * CELL_SIZE + CELL_SIZE / 2.0;
                    marker.pose.position.y = -(y * CELL_SIZE);
                    marker.pose.position.z = WALL_HEIGHT / 2.0;
                    marker.pose.orientation = no_rotation;
                    marker.scale = wall_scale;
                    marker.color = wall_color;
                    msg.markers.push_back(marker);
                }
            }
        }

        // posts
        for (size_t x = 0; x <= map_->get_height(); ++x) {
            for (size_t y = 0; y <= map_->get_width(); ++y) {
                auto marker = visualization_msgs::msg::Marker();
                marker.header = header;
                marker.ns = "maze";
                marker.id = marker_id++;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = x * CELL_SIZE;
                marker.pose.position.y = -(y * CELL_SIZE);
                marker.pose.position.z = WALL_HEIGHT / 2.0;
                marker.pose.orientation = no_rotation;
                marker.scale.x = WALL_WIDTH - 0.001;
                marker.scale.y = WALL_WIDTH - 0.001;
                marker.scale.z = WALL_HEIGHT;
                marker.color = post_color;
                msg.markers.push_back(marker);
            }
        }

        maze_markers_pub_->publish(msg);
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        maze_markers_pub_;

    std::string map_path_;
    std::shared_ptr<OnlineFileMap> map_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapVisualizer>());
    rclcpp::shutdown();
    return 0;
}
