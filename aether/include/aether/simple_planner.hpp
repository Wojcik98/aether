#ifndef _AETHER_INCLUDE_SIMPLE_PLANNER_HPP
#define _AETHER_INCLUDE_SIMPLE_PLANNER_HPP

#include "aether/planner.hpp"
#include "aether/robot_config.hpp"
#include "aether/types.hpp"

/*
 * SimplePlanner moves the robot with a constant linear velocity and simple
 * arc turns.
 */
class SimplePlanner : public Planner<SimplePlanner> {
public:
    SimplePlanner(Path &path, float vel_lin, float vel_ang)
        : vel_lin_(vel_lin), vel_ang_(vel_ang), path_(path) {}

    void reparse_path();
    void start(const FullState &state);
    FullState get_next_state();
    bool goal_reached();

private:
    enum class AbsDirection : uint8_t {
        NORTH = 0,
        WEST = 1,
        SOUTH = 2,
        EAST = 3,
        STOP = 4,
    };

    enum class RelDirection : uint8_t {
        FORWARD = 0,
        LEFT = 1,
        RIGHT = 2,
        BACKWARD = 3,
        STOP = 4,
    };

    static constexpr float dt = 1.0f / FREQ_CONTROL;
    const float vel_lin_;
    const float vel_ang_;
    FullState state_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    size_t path_idx_ = 0;
    Path &path_;
    bool goal_reached_ = false;

    float x0_;
    float cos_th0_;
    float y0_;
    float sin_th0_;
    float th0_;
    float om_z_;
    float T_;
    float t_;
    float rot_center_x_;
    float rot_center_y_;
    RelDirection turn_dir_;
    AbsDirection current_abs_dir_ = AbsDirection::NORTH;
    void calculate_turn_params(const CellCoords &current_cell,
                               AbsDirection prev_dir, AbsDirection current_dir);

    AbsDirection get_abs_dir(const CellCoords &current, const CellCoords &next);
    RelDirection get_rel_dir(const AbsDirection &prev,
                             const AbsDirection &current);
    AbsDirection get_state_dir(const FullState &state);
};

#endif // _AETHER_INCLUDE_SIMPLE_PLANNER_HPP