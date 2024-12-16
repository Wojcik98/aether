#ifndef _AETHER_INCLUDE_MAP_INTERFACE_HPP_
#define _AETHER_INCLUDE_MAP_INTERFACE_HPP_

#include <cmath>
#include <cstdint>

#include "aether/robot_config.hpp"
#include "aether/types.hpp"

// Use the Curiously Recurring Template Pattern (CRTP) to allow static
// polymorphism. This way, I can have a single interface for both MapInProgress
// and MapMock for testing.

template <class T> struct MapInterface {
    static CellCoords get_cell_coords(float x, float y) {
        x = floorf(x / CELL_SIZE);
        y = floorf(y / CELL_SIZE);
        return {
            static_cast<int32_t>(x),
            static_cast<int32_t>(y),
        };
    }

    CellWalls get_cell_walls(float x, float y) const {
        auto [x_cell, y_cell] = get_cell_coords(x, y);
        return get_cell_walls(x, y);
    }

    CellWalls get_cell_walls(int32_t x, int32_t y) const {
        return static_cast<const T *>(this)->get_cell_walls(x, y);
    }
};
#endif // _AETHER_INCLUDE_MAP_INTERFACE_HPP_
