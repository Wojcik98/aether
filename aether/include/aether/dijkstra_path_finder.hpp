#ifndef _AETHER_INCLUDE_DIJKSTRA_PATH_FINDER_HPP_
#define _AETHER_INCLUDE_DIJKSTRA_PATH_FINDER_HPP_

#include <array>
#include <cstdint>

#include "aether/map_confident.hpp"
#include "aether/path_finder.hpp"
#include "aether/robot_config.hpp"
#include "aether/types.hpp"

template <class MapImpl>
class DijkstraPathFinder : public PathFinder<DijkstraPathFinder<MapImpl>> {
public:
    DijkstraPathFinder(MapConfident<MapImpl> &map) : map_(map) {
        this->path.at(0) = {0, -1};
        this->path.at(1) = {0, -1};
    }

    void find_path(const CellCoords &start, const CellCoords &goal) {
        constexpr uint32_t COST = 1;
        // reset visited, frontier, cost arrays
        for (uint32_t i = 0; i < MAZE_SIZE_X_CELLS; ++i) {
            visited_[i].fill(false);
            frontier_[i].fill(false);
            cost_[i].fill(UINT32_MAX);
        }

        // initialize
        CellCoords current = start;
        auto [x, y] = current;
        uint32_t y_arr = -y - 1;
        cost_[x][y_arr] = 0;
        frontier_[x][y_arr] = true;

        // Dijkstra's algorithm
        // Complexity: O(V^2)
        while (true) {
            if (current == goal) {
                break;
            }

            auto [x, y] = current;
            uint32_t y_arr = -y - 1;
            visited_[x][y_arr] = true;
            frontier_[x][y_arr] = false;

            // update costs of neighbors
            auto walls = map_.get_cell_walls(x, y);
            if (!walls.north && !visited_[x + 1][y_arr]) {
                cost_[x + 1][y_arr] =
                    std::min(cost_[x + 1][y_arr], cost_[x][y_arr] + COST);
                frontier_[x + 1][y_arr] = true;
            }
            if (!walls.west && !visited_[x][y_arr - 1]) {
                cost_[x][y_arr - 1] =
                    std::min(cost_[x][y_arr - 1], cost_[x][y_arr] + COST);
                frontier_[x][y_arr - 1] = true;
            }
            if (!walls.south && !visited_[x - 1][y_arr]) {
                cost_[x - 1][y_arr] =
                    std::min(cost_[x - 1][y_arr], cost_[x][y_arr] + COST);
                frontier_[x - 1][y_arr] = true;
            }
            if (!walls.east && !visited_[x][y_arr + 1]) {
                cost_[x][y_arr + 1] =
                    std::min(cost_[x][y_arr + 1], cost_[x][y_arr] + COST);
                frontier_[x][y_arr + 1] = true;
            }

            // find the next cell to visit
            uint32_t min_cost = UINT32_MAX;
            CellCoords next = current;
            for (int32_t i = 0; i < (int32_t)MAZE_SIZE_X_CELLS; ++i) {
                for (int32_t j = 0; j < (int32_t)MAZE_SIZE_Y_CELLS; ++j) {
                    if (frontier_[i][j] && cost_[i][j] < min_cost) {
                        min_cost = cost_[i][j];
                        next = {i, -j - 1};
                    }
                }
            }

            if (next == current) {
                // no path found
                break;
            }

            current = next;
        }

        // backtracking
        this->path[0] = goal;
        current = goal;
        int32_t idx = 1;
        while (current != start) {
            auto [x, y] = current;
            uint32_t y_arr = -y - 1;
            uint32_t min_cost = cost_[x][y_arr];

            CellCoords next = current;
            auto walls = map_.get_cell_walls(x, y);
            if (!walls.north && cost_[x + 1][y_arr] < min_cost) {
                min_cost = cost_[x + 1][y_arr];
                next = {x + 1, y};
            }
            if (!walls.west && cost_[x][y_arr - 1] < min_cost) {
                min_cost = cost_[x][y_arr - 1];
                next = {x, y + 1};
            }
            if (!walls.south && cost_[x - 1][y_arr] < min_cost) {
                min_cost = cost_[x - 1][y_arr];
                next = {x - 1, y};
            }
            if (!walls.east && cost_[x][y_arr + 1] < min_cost) {
                min_cost = cost_[x][y_arr + 1];
                next = {x, y - 1};
            }

            this->path[idx++] = next;
            if (next == current) {
                break;
            }
            current = next;
        }

        // reverse the path
        std::reverse(this->path.begin(), this->path.begin() + idx);
        this->path[idx] = this->path[idx - 1];
    }

private:
    MapConfident<MapImpl> &map_;
    std::array<std::array<bool, MAZE_SIZE_Y_CELLS>, MAZE_SIZE_X_CELLS> visited_;
    std::array<std::array<bool, MAZE_SIZE_Y_CELLS>, MAZE_SIZE_X_CELLS>
        frontier_;
    std::array<std::array<uint32_t, MAZE_SIZE_Y_CELLS>, MAZE_SIZE_X_CELLS>
        cost_;
};
#endif // _AETHER_INCLUDE_DIJKSTRA_PATH_FINDER_HPP_