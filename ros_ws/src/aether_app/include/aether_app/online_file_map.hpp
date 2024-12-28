#ifndef _AETHER_APP_ONLINE_FILE_MAP_HPP_
#define _AETHER_APP_ONLINE_FILE_MAP_HPP_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <aether/map_interface.hpp>
#include <aether/mapped_localization.hpp>
#include <aether/robot_config.hpp>

using HorizontalWalls =
    std::array<std::array<float, MAZE_SIZE_Y_CELLS + 1>, MAZE_SIZE_X_CELLS>;
using VerticalWalls =
    std::array<std::array<float, MAZE_SIZE_Y_CELLS>, MAZE_SIZE_X_CELLS + 1>;

/*
 * Loads map from a file in runtime.
 */
class OnlineFileMap : public MapInterface<OnlineFileMap> {
public:
    OnlineFileMap(const std::string &map_path) : map_path_(map_path) {
        // initially fill the walls with zeros
        for (auto &row : horizontal_walls_) {
            row.fill(0.0f);
        }
        for (auto &row : vertical_walls_) {
            row.fill(0.0f);
        }

        std::ifstream map_file(map_path_);
        if (!map_file.is_open()) {
            throw std::runtime_error("Could not open map file");
        }

        std::vector<std::string> map_lines;
        while (map_file) {
            std::string line;
            std::getline(map_file, line);
            map_lines.push_back(line);
        }

        height_ = (map_lines.size() - 1) / 2;
        width_ = (map_lines[0].size() - 1) / 2;

        std::cout << "height: " << height_ << " width: " << width_ << std::endl;

        // horizontal walls
        for (std::size_t x = 0; x <= height_; ++x) {
            for (std::size_t y = 0; y < width_; ++y) {
                int file_row = 2 * (height_ - x);
                int file_col = 2 * y + 1;

                horizontal_walls_[x][y] =
                    static_cast<float>(map_lines[file_row][file_col] == 'W');
            }
        }

        // vertical walls
        for (std::size_t x = 0; x < height_; ++x) {
            for (std::size_t y = 0; y <= width_; ++y) {
                int file_row = 2 * (height_ - x) - 1;
                int file_col = 2 * y;
                vertical_walls_[x][y] =
                    static_cast<float>(map_lines[file_row][file_col] == 'W');
            }
        }
    }

    CellWalls get_cell_walls(int32_t x, int32_t y) const {
        // bottom left corner is (0, -1) (maze goes to the negative y direction)
        // we need to negate the y coordinate and offset to index arrays
        y = -y - 1;
        return {
            horizontal_walls_[x + 1][y], // north
            vertical_walls_[x][y],       // west
            horizontal_walls_[x][y],     // south
            vertical_walls_[x][y + 1],   // east
        };
    }

    bool is_out_of_bounds(float x, float y) const {
        const float top_height = height_ * CELL_SIZE;
        const float top_width = -(width_ * CELL_SIZE);
        return x < 0.0f || x > top_height || y > 0.0f || y < top_width;
    }

    std::size_t get_height() const { return height_; }

    std::size_t get_width() const { return width_; }

    const HorizontalWalls &get_horizontal_walls() const {
        return horizontal_walls_;
    }

    const VerticalWalls &get_vertical_walls() const { return vertical_walls_; }

private:
    std::string map_path_;

    // The walls are stored in the following way:
    // horizontal_walls_[x][y] is the wall below cell (x, -y)
    // horizontal_walls_[x + 1][y] is the wall above cell (x, -y)
    // vertical_walls_[x][y] is the wall to the left of cell (x, -y)
    // vertical_walls_[x][y + 1] is the wall to the right of cell (x, -y)
    // The walls are stored as probabilities that the wall is there.
    // IMPORTNANT: The walls are stored in the negative y direction!
    HorizontalWalls horizontal_walls_;
    VerticalWalls vertical_walls_;

    std::size_t height_;
    std::size_t width_;
};

#endif // _AETHER_APP_ONLINE_FILE_MAP_HPP_
