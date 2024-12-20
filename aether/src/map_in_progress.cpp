#include "aether/map_in_progress.hpp"

CellWalls MapInProgress::get_cell_walls(int32_t x, int32_t y) const {
    y = -y; // indexing with positive y values
    return {
        horizontal_walls_[x + 1][y], // north
        vertical_walls_[x][y],       // west
        horizontal_walls_[x][y],     // south
        vertical_walls_[x][y + 1],   // east
    };
}

bool MapInProgress::is_out_of_bounds(float x, float y) const {
    return x < 0.0f || x > MAZE_SIZE_X || y > 0.0f || y < -MAZE_SIZE_Y;
}
