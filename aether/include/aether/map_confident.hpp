#ifndef _AETHER_INCLUDE_MAP_CONFIDENT_HPP_
#define _AETHER_INCLUDE_MAP_CONFIDENT_HPP_

#include <cmath>

#include "aether/map_interface.hpp"
#include "aether/robot_config.hpp"

template <class MapImpl> class MapConfident {
public:
    static constexpr float CONFIDENCE_THRESHOLD = 0.5f;
    MapConfident(MapInterface<MapImpl> &map) : map_(map) {}

    /**
     * @brief Check if there is an obstacle at the given point (close to the
     * potential obstacle).
     *
     * @param x x coordinate of the point.
     * @param y y coordinate of the point.
     */
    bool is_obstacle(float x, float y) const {
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

private:
    MapInterface<MapImpl> &map_;
};

#endif // _AETHER_INCLUDE_MAP_CONFIDENT_HPP_
