#ifndef _AETHER_INCLUDE_MAP_CONFIDENT_HPP_
#define _AETHER_INCLUDE_MAP_CONFIDENT_HPP_

#include "map_in_progress.hpp"

class MapConfident {
public:
    MapConfident(MapInProgress &map) : map_(map) {}

    float probability(float x, float y, float std) const;

private:
    MapInProgress &map_;
};

#endif // _AETHER_INCLUDE_MAP_CONFIDENT_HPP_
