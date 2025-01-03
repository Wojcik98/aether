#ifndef _AETHER_INCLUDE_CONTROLLER_HPP_
#define _AETHER_INCLUDE_CONTROLLER_HPP_

#include "aether/robot_config.hpp"
#include "aether/types.hpp"

class Controller {
public:
    Controller() = default;

    Twist get_twist(const FullState &current, const FullState &target);

    void set_KPx(float KPx) { this->KPx = KPx; }
    void set_KPy(float KPy) { this->KPy = KPy; }
    void set_KPom(float KPom) { this->KPom = KPom; }

private:
    float KPx = 1.0f;
    float KPy = 1.0f;
    float KPom = 1.0f;
};

#endif // _AETHER_INCLUDE_CONTROLLER_HPP_
