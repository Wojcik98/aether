#include <gtest/gtest.h>

#include "aether/map_confident.hpp"
#include "aether/map_interface.hpp"
#include "aether/mapped_localization.hpp"
#include "aether/robot_config.hpp"
#include "aether/types.hpp"

class MapMock : public MapInterface<MapMock> {
public:
    static CellCoords get_cell_coords(float x, float y) { return {0, 0}; }

    CellWalls get_cell_walls(float x, float y) const {
        return {0.0f, 0.0f, 0.0f, 0.0f};
    }

    CellWalls get_cell_walls(int8_t x, int8_t y) const {
        return {0.0f, 0.0f, 0.0f, 0.0f};
    }
};

TEST(LocalizationTest, Dummy) {
    MapMock map;
    MapConfident<MapMock> map_confident(map);

    MappedLocalization<MapMock> localization(map_confident);

    auto pose = localization.get_latest_pose();
    EXPECT_FLOAT_EQ(pose.x, STARTING_X);
    EXPECT_FLOAT_EQ(pose.y, STARTING_Y);
    EXPECT_FLOAT_EQ(pose.yaw, STARTING_YAW);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
