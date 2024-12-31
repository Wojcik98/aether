#include "aether/controller.hpp"

#include <cmath>

#include "aether/robot_config.hpp"
#include "aether/types.hpp"

Twist Controller::get_twist(const FullState &current, const FullState &target) {
    auto err = target - current;
    Twist twist;

    float err_dist = sqrtf(err.x * err.x + err.y * err.y);
    float err_angle = atan2f(err.y, err.x) - current.yaw;

    twist.vx = target.vx + KPx * err_dist;
    twist.omega = target.omega + KPom * err_angle;

    return twist;
}
