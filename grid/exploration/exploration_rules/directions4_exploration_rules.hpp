#pragma once

#include "grid/grid_types.hpp"
#include "direction/direction.hpp"
#include "direction/neighbourhood.hpp"

struct Directions4_exploration_rules
{
    using Direction = Direction4;

    template <typename GridWorld, typename Mark>
    inline constexpr const auto& move_actions(const GridWorld&, const Position&, const Mark&) const
    {
        return Directions4::directions;
    }

    inline Position next_position(const Position& position, Direction4 dir) const
    {
        return neighbour(position, dir);
    }

    inline Position previous_position(const Position& position, Direction4 dir) const
    {
        return neighbour(position, dir.opposed());
    }
};
