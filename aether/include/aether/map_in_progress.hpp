#ifndef _AETHER_INCLUDE_MAP_IN_PROGRESS_HPP_
#define _AETHER_INCLUDE_MAP_IN_PROGRESS_HPP_

#include <array>
#include <cstdint>
#include <tuple>

#include "aether/map_interface.hpp"
#include "aether/robot_config.hpp"
#include "aether/types.hpp"

// Orientation convention:
// (0, 0) is the middle of the most bottom left post of the maze.
// x is the vertical axis, y is the horizontal axis, as in the Cartesian plane.
// IMPORTANT: This means that y values are always negative!
// North is the positive x direction, west is the positive y direction.
class MapInProgress : public MapInterface<MapInProgress> {
public:
    MapInProgress() = default;

    static CellCoords get_cell_coords(float x, float y);

    CellWalls get_cell_walls(float x, float y) const;
    CellWalls get_cell_walls(int8_t x, int8_t y) const;

private:
    // The walls are stored in the following way:
    // horizontal_walls_[x][y] is the wall below cell (x, -y)
    // horizontal_walls_[x + 1][y] is the wall above cell (x, -y)
    // vertical_walls_[x][y] is the wall to the left of cell (x, -y)
    // vertical_walls_[x][y + 1] is the wall to the right of cell (x, -y)
    // The walls are stored as probabilities that the wall is there.
    // IMPORTNANT: The walls are stored in the negative y direction!
    std::array<std::array<float, MAZE_SIZE_Y + 1>, MAZE_SIZE_X>
        horizontal_walls_;
    std::array<std::array<float, MAZE_SIZE_Y>, MAZE_SIZE_X + 1> vertical_walls_;
};

#endif // _AETHER_INCLUDE_MAP_IN_PROGRESS_HPP_
