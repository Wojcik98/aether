#include "aether/controller.hpp"

#include <cmath>

#include "aether/robot_config.hpp"
#include "aether/types.hpp"

Twist Controller::get_twist(const FullState &current, const FullState &target) {
    // calculate target frame in current frame
    auto target_local = current.to_pose().inverse() * target.to_pose();

    if (target_local.yaw > PI_F) {
        target_local.yaw -= 2 * PI_F;
    } else if (target_local.yaw < -PI_F) {
        target_local.yaw += 2 * PI_F;
    }

    Twist twist;
    twist.vx = target.vx + KPx * target_local.x;
    twist.omega = target.omega + KPom * target_local.yaw + KPy * target_local.y;
    return twist;
}
