#ifndef _AETHER_INCLUDE_MAP_INTERFACE_HPP_
#define _AETHER_INCLUDE_MAP_INTERFACE_HPP_

#include <cmath>
#include <cstdint>

#include "aether/robot_config.hpp"
#include "aether/types.hpp"

namespace aether {
namespace map {
CellCoords get_cell_coords(float x, float y) {
    x = floorf(x / CELL_SIZE);
    y = floorf(y / CELL_SIZE);
    return {
        static_cast<int32_t>(x),
        static_cast<int32_t>(y),
    };
}

/**
 * @brief Get relative coordinates of the point in the cell, (0, 0) is the
 * middle of the cell.
 */
RelCoords get_relative_coords(float x, float y) {
    auto [x_cell, y_cell] = get_cell_coords(x, y);
    float x_rel = x - x_cell * CELL_SIZE - CELL_SIZE / 2.0f;
    float y_rel = y - y_cell * CELL_SIZE - CELL_SIZE / 2.0f;
    return {x_rel, y_rel};
}

} // namespace map
} // namespace aether

// Use the Curiously Recurring Template Pattern (CRTP) to allow static
// polymorphism. This way, I can have a single interface for both MapInProgress
// and MapMock for testing.

template <class T> struct MapInterface {
    CellWalls get_cell_walls(int32_t x, int32_t y) const {
        return static_cast<const T *>(this)->get_cell_walls(x, y);
    }

    bool is_out_of_bounds(float x, float y) const {
        return static_cast<const T *>(this)->is_out_of_bounds(x, y);
    }
};

#endif // _AETHER_INCLUDE_MAP_INTERFACE_HPP_
