#include "aether/map_confident.hpp"

#include <cmath>

#include "aether/map_in_progress.hpp"
#include "aether/robot_config.hpp"

bool MapConfident::is_obstacle(float x, float y) const {
    constexpr float EPS = 0.003f;
    constexpr float WALL_THRES = CELL_INNER_SIZE / 2.0f - EPS;
    auto [x_cell, y_cell] = map_.get_cell_coords(x, y);
    auto walls = map_.get_cell_walls(x_cell, y_cell);
    // relative coordinates, (0, 0) is the middle of the cell
    float x_rel = x - x_cell * CELL_SIZE - CELL_SIZE / 2.0f;
    float y_rel = y - y_cell * CELL_SIZE - CELL_SIZE / 2.0f;

    // check posts in the corners
    if (abs(x_rel) > WALL_THRES && abs(y_rel) > WALL_THRES) {
        // posts always present
        return true;
    } // now we are sure only one of the coordinates is close to the wall
    else if (x_rel > WALL_THRES) {
        // check north wall
        return walls.north > CONFIDENCE_THRESHOLD;
    } else if (x_rel < -WALL_THRES) {
        // check south wall
        return walls.south > CONFIDENCE_THRESHOLD;
    } else if (y_rel > WALL_THRES) {
        // check west wall
        return walls.west > CONFIDENCE_THRESHOLD;
    } else if (y_rel < -WALL_THRES) {
        // check east wall
        return walls.east > CONFIDENCE_THRESHOLD;
    }

    return false;
}
