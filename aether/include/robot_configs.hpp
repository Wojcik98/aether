#ifndef _AETHER_INCLUDE_ROBOT_CONFIG_HPP_
#define _AETHER_INCLUDE_ROBOT_CONFIG_HPP_

constexpr float PI_F = 3.14159265358979f;
constexpr float PI_2_F = 1.57079632679490f;

constexpr float ROBOT_LENGTH = 0.18f;
constexpr float MOTORS_OFFSET = 0.06f; // Distance from the back to the motors.
constexpr float ROBOT_WIDTH = 0.12f;
constexpr float WHEEL_RADIUS = 0.03f;
constexpr float WHEEL_BASE = 0.12f;

constexpr float WALL_WIDTH = 0.012f;
constexpr float CELL_SIZE = 0.18f;

// Starting position of the robot's base (middlepoint between motors).
// The robot starts in the bottom left corner of the map, in the center of the
// cell, fully backed up against the wall.
constexpr float STARTING_X = WALL_WIDTH + MOTORS_OFFSET;
constexpr float STARTING_Y = -CELL_SIZE / 2.0f;
constexpr float STARTING_YAW = 0.0f;

#endif // _AETHER_INCLUDE_ROBOT_CONFIG_HPP_
