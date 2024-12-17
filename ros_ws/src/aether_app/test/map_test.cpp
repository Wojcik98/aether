#include <gtest/gtest.h>

#include "aether/map_confident.hpp"
#include "aether/map_interface.hpp"

class MapMock : public MapInterface<MapMock> {
public:
    CellWalls get_cell_walls(int32_t x, int32_t y) const {
        (void)x;
        (void)y;
        return {0.0f, 0.0f, 0.0f, 0.0f};
    }

    bool is_out_of_bounds(float x, float y) const {
        (void)x;
        (void)y;
        return false;
    }
};

TEST(MapObstaclesTest, Dummy) {
    MapMock map;
    MapConfident<MapMock> map_confident(map);

    // corners are always obstacles
    EXPECT_EQ(map_confident.is_obstacle(0.0f, 0.0f), true);

    // middle of the cell is never an obstacle
    EXPECT_EQ(map_confident.is_obstacle(CELL_SIZE / 2.0f, CELL_SIZE / 2.0f),
              false);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
