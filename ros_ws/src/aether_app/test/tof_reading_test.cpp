#include <gtest/gtest.h>

#include "aether/map_confident.hpp"
#include "aether/map_interface.hpp"
#include "aether/mapped_localization.hpp"
#include "aether/robot_config.hpp"
#include "aether/types.hpp"

class CorridorMapMock : public MapInterface<CorridorMapMock> {
public:
    CellWalls get_cell_walls(int8_t x, int8_t y) const {
        (void)x;
        (void)y;
        return {0.0f, 1.0f, 0.0f, 1.0f};
    }
};

class TofMapTest : public ::testing::Test {
protected:
    TofMapTest() : map_confident(map), localization(map_confident) {
        localization.set_random_seed(0);
    }

    CorridorMapMock map;
    MapConfident<CorridorMapMock> map_confident;
    MappedLocalization<CorridorMapMock> localization;
};
