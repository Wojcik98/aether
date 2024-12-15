#include "aether/map_in_progress.hpp"

CellWalls MapInProgress::get_cell_walls(int8_t x, int8_t y) const {
    y = -y; // indexing with positive y values
    return {
        horizontal_walls_[x + 1][y], // north
        vertical_walls_[x][y],       // west
        horizontal_walls_[x][y],     // south
        vertical_walls_[x][y + 1],   // east
    };
}
