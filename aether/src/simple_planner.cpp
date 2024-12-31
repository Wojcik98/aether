#include "aether/simple_planner.hpp"

#include <array>
#include <cmath>
#include <string>

static std::array<std::string, 5> abs_dir_str = {"NORTH", "WEST", "SOUTH",
                                                 "EAST", "STOP"};
static std::array<std::string, 5> rel_dir_str = {"FORWARD", "LEFT", "RIGHT",
                                                 "BACKWARD", "STOP"};

void SimplePlanner::reparse_path() {}

void SimplePlanner::start(const FullState &state) {
    state_ = state;
    path_idx_ = 0;
    goal_reached_ = false;

    // auto state_cell = aether::map::get_cell_coords(state_.x, state_.y);
    // auto path_start_cell = path_.at(0);

    // if (state_cell != path_start_cell) {
    //     // TODO: handle error
    // }

    auto start_dir = get_state_dir(state_);
    auto &current_cell = path_.at(0);
    current_abs_dir_ = get_abs_dir(current_cell, path_.at(1));
    calculate_turn_params(current_cell, start_dir, current_abs_dir_);
    t_ = T_ / 2; // TODO: calculate initial t
}

FullState SimplePlanner::get_next_state() {
    if (t_ + dt > T_) {
        if (current_abs_dir_ == AbsDirection::STOP) {
            goal_reached_ = true;
            state_.x = x0_ + (CELL_SIZE / 2.0f) * cos_th0_;
            state_.y = y0_ + (CELL_SIZE / 2.0f) * sin_th0_;
            state_.vx = 0.0f;
            state_.omega = 0.0f;
            return state_;
        }
        t_ = t_ + dt - T_;
        path_idx_++; // should never go out of bounds
        auto prev_dir = current_abs_dir_;
        current_abs_dir_ =
            get_abs_dir(path_.at(path_idx_), path_.at(path_idx_ + 1));
        calculate_turn_params(path_.at(path_idx_), prev_dir, current_abs_dir_);
    } else {
        t_ += dt;
    }

    state_.vx = vel_lin_;
    state_.omega = om_z_;
    switch (turn_dir_) {
    case RelDirection::FORWARD:
    case RelDirection::STOP:
        state_.yaw = th0_;
        state_.x = x0_ + t_ * vel_lin_ * cos_th0_;
        state_.y = y0_ + t_ * vel_lin_ * sin_th0_;
        break;
    case RelDirection::LEFT:
        state_.yaw = th0_ + om_z_ * t_;
        state_.x = rot_center_x_ + (x0_ - rot_center_x_) * cosf(om_z_ * t_) -
                   (y0_ - rot_center_y_) * sinf(om_z_ * t_);
        state_.y = rot_center_y_ + (x0_ - rot_center_x_) * sinf(om_z_ * t_) +
                   (y0_ - rot_center_y_) * cosf(om_z_ * t_);
        break;
    case RelDirection::RIGHT:
        state_.yaw = th0_ + om_z_ * t_;
        state_.x = rot_center_x_ + (x0_ - rot_center_x_) * cosf(-om_z_ * t_) +
                   (y0_ - rot_center_y_) * sinf(-om_z_ * t_);
        state_.y = rot_center_y_ - (x0_ - rot_center_x_) * sinf(-om_z_ * t_) +
                   (y0_ - rot_center_y_) * cosf(-om_z_ * t_);
        break;
    case RelDirection::BACKWARD:
        state_.vx = 0.0f;
        state_.yaw = th0_ + t_ * state_.omega;
        state_.x = x0_;
        state_.y = y0_;
        break;
    }

    return state_;
}

void SimplePlanner::calculate_turn_params(const CellCoords &current_cell,
                                          AbsDirection prev_dir,
                                          AbsDirection current_dir) {
    float cell_center_x =
        std::get<0>(current_cell) * CELL_SIZE + CELL_SIZE / 2.0f;
    float cell_center_y =
        std::get<1>(current_cell) * CELL_SIZE + CELL_SIZE / 2.0f;

    switch (prev_dir) {
    case AbsDirection::NORTH:
        x0_ = cell_center_x - CELL_SIZE / 2.0f;
        y0_ = cell_center_y;
        th0_ = 0.0f;
        break;
    case AbsDirection::WEST:
        x0_ = cell_center_x;
        y0_ = cell_center_y - CELL_SIZE / 2.0f;
        th0_ = PI_2_F;
        break;
    case AbsDirection::SOUTH:
        x0_ = cell_center_x + CELL_SIZE / 2.0f;
        y0_ = cell_center_y;
        th0_ = PI_F;
        break;
    case AbsDirection::EAST:
        x0_ = cell_center_x;
        y0_ = cell_center_y + CELL_SIZE / 2.0f;
        th0_ = -PI_2_F;
        break;
    case AbsDirection::STOP:
        // Should never happen
        break;
    }

    cos_th0_ = cosf(th0_);
    sin_th0_ = sinf(th0_);

    turn_dir_ = get_rel_dir(prev_dir, current_dir);
    switch (turn_dir_) {
    case RelDirection::FORWARD:
        T_ = CELL_SIZE / vel_lin_;
        om_z_ = 0.0f;
        break;
    case RelDirection::LEFT:
        T_ = PI_F * CELL_SIZE / (4.0f * vel_lin_);
        om_z_ = PI_2_F / T_;
        rot_center_x_ = x0_ - sin_th0_ * CELL_SIZE / 2.0f;
        rot_center_y_ = y0_ + cos_th0_ * CELL_SIZE / 2.0f;
        break;
    case RelDirection::RIGHT:
        T_ = PI_F * CELL_SIZE / (4.0f * vel_lin_);
        om_z_ = -PI_2_F / T_;
        rot_center_x_ = x0_ + sin_th0_ * CELL_SIZE / 2.0f;
        rot_center_y_ = y0_ - cos_th0_ * CELL_SIZE / 2.0f;
        break;
    case RelDirection::BACKWARD:
        T_ = PI_F / vel_ang_;
        om_z_ = vel_ang_;
        break;
    case RelDirection::STOP:
        T_ = CELL_SIZE / (2.0f * vel_lin_);
        om_z_ = 0.0f;
    }
}

SimplePlanner::AbsDirection
SimplePlanner::get_abs_dir(const CellCoords &current, const CellCoords &next) {
    int8_t dx = std::get<0>(next) - std::get<0>(current);
    int8_t dy = std::get<1>(next) - std::get<1>(current);

    if (dx == 1) {
        return AbsDirection::NORTH;
    } else if (dx == -1) {
        return AbsDirection::SOUTH;
    } else if (dy == 1) {
        return AbsDirection::WEST;
    } else if (dy == -1) {
        return AbsDirection::EAST;
    } else if (dx == 0 && dy == 0) {
        return AbsDirection::STOP;
    } else {
        // Should never happen
        return AbsDirection::NORTH;
    }
}

SimplePlanner::RelDirection
SimplePlanner::get_rel_dir(const AbsDirection &prev,
                           const AbsDirection &current) {
    using DirectionMapping = std::array<std::array<RelDirection, 5>, 4>;
    static constexpr DirectionMapping mapping = {{
        // prev = NORTH
        {RelDirection::FORWARD, RelDirection::LEFT, RelDirection::BACKWARD,
         RelDirection::RIGHT, RelDirection::STOP},
        // prev = WEST
        {RelDirection::RIGHT, RelDirection::FORWARD, RelDirection::LEFT,
         RelDirection::BACKWARD, RelDirection::STOP},
        // prev = SOUTH
        {RelDirection::BACKWARD, RelDirection::RIGHT, RelDirection::FORWARD,
         RelDirection::LEFT, RelDirection::STOP},
        // prev = EAST
        {RelDirection::LEFT, RelDirection::BACKWARD, RelDirection::RIGHT,
         RelDirection::FORWARD, RelDirection::STOP},
    }};
    return mapping[static_cast<uint8_t>(prev)][static_cast<uint8_t>(current)];
}

SimplePlanner::AbsDirection
SimplePlanner::get_state_dir(const FullState &state) {
    constexpr float deg45 = PI_2_F / 2.0f;
    float yaw = state.yaw;
    // normalize to [-pi, pi]
    while (yaw > PI_F) {
        yaw -= 2 * PI_F;
    }
    while (yaw < -PI_F) {
        yaw += 2 * PI_F;
    }

    if (abs(yaw) < deg45) {
        return AbsDirection::NORTH;
    } else if (abs(yaw) > PI_F - deg45) {
        return AbsDirection::SOUTH;
    } else if (yaw > 0) {
        return AbsDirection::WEST;
    } else {
        return AbsDirection::EAST;
    }
}

bool SimplePlanner::goal_reached() { return goal_reached_; }
