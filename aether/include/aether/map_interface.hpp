#ifndef _AETHER_INCLUDE_MAP_INTERFACE_HPP_
#define _AETHER_INCLUDE_MAP_INTERFACE_HPP_

#include <cstdint>

#include "aether/robot_config.hpp"
#include "aether/types.hpp"

// Use the Curiously Recurring Template Pattern (CRTP) to allow static
// polymorphism. This way, I can have a single interface for both MapInProgress
// and MapMock for testing.

template <class T> struct MapInterface {
    static CellCoords get_cell_coords(float x, float y) {
        return {
            static_cast<int8_t>(x / CELL_SIZE),
            static_cast<int8_t>(y / CELL_SIZE),
        };
    }

    CellWalls get_cell_walls(float x, float y) const {
        return static_cast<const T *>(this)->get_cell_walls(x, y);
    }

    CellWalls get_cell_walls(int8_t x, int8_t y) const {
        return static_cast<const T *>(this)->get_cell_walls(x, y);
    }
};
#endif // _AETHER_INCLUDE_MAP_INTERFACE_HPP_
