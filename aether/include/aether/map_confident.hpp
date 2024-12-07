#ifndef _AETHER_INCLUDE_MAP_CONFIDENT_HPP_
#define _AETHER_INCLUDE_MAP_CONFIDENT_HPP_

#include "map_in_progress.hpp"

class MapConfident {
public:
    static constexpr float CONFIDENCE_THRESHOLD = 0.5f;
    MapConfident(MapInProgress &map) : map_(map) {}

    /**
     * @brief Check if there is an obstacle at the given point (close to the
     * potential obstacle).
     *
     * @param x x coordinate of the point.
     * @param y y coordinate of the point.
     */
    bool is_obstacle(float x, float y) const;

private:
    MapInProgress &map_;
};

#endif // _AETHER_INCLUDE_MAP_CONFIDENT_HPP_
