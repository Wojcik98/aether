#ifndef _AETHER_INCLUDE_DUMMY_PATH_FINDER_HPP_
#define _AETHER_INCLUDE_DUMMY_PATH_FINDER_HPP_

#include "aether/path_finder.hpp"
#include "aether/robot_config.hpp"
#include "aether/types.hpp"

class DummyPathFinder : public PathFinder<DummyPathFinder> {
public:
    DummyPathFinder() {
        path.at(0) = {0, -1};
        path.at(1) = {1, -1};
        path.at(2) = {1, -2};
        path.at(3) = {1, -3};
        path.at(4) = {2, -3};
        path.at(5) = {2, -3};
    }

    void find_path(const CellCoords &start, const CellCoords &goal) {
        // Dummy path
        (void)start;
        (void)goal;
    }
};

#endif // _AETHER_INCLUDE_DUMMY_PATH_FINDER_HPP_