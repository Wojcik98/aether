#ifndef _AETHER_INCLUDE_PATH_FINDER_HPP
#define _AETHER_INCLUDE_PATH_FINDER_HPP

#include "aether/types.hpp"

// Use the Curiously Recurring Template Pattern (CRTP) to allow static
// polymorphism.

template <class T> struct PathFinder {
    Path path;

    void find_path(const CellCoords &start, const CellCoords &goal) {
        return static_cast<const T *>(this)->find_path(start, goal);
    }
};

#endif // _AETHER_INCLUDE_PATH_FINDER_HPP