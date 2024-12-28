#ifndef _AETHER_INCLUDE_PLANNER_HPP
#define _AETHER_INCLUDE_PLANNER_HPP

#include "aether/types.hpp"

// Use the Curiously Recurring Template Pattern (CRTP) to allow static
// polymorphism.

template <class T> struct Planner {
    void reparse_path() { static_cast<const T *>(this)->reparse_path(); }

    void start(FullState &state) { static_cast<const T *>(this)->start(state); }

    FullState get_next_state() {
        return static_cast<const T *>(this)->get_next_state();
    }

    bool goal_reached() { return static_cast<const T *>(this)->goal_reached(); }
};

#endif // _AETHER_INCLUDE_PLANNER_HPP