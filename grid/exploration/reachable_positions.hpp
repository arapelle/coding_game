#pragma once

#include "spread_exploration.hpp"

/**
 * reachable_positions(grid, pos, accessibility_test, radius) -> vector<Position>
 */
template <typename GridWorld, typename AccessibilityTest, typename ExplorationRules = Default>
Position_set
reachable_positions(const GridWorld& world, const Position& start, ExplorationRules exploration_rules,
                    AccessibilityTest accessibility_test,
                    unsigned radius = std::numeric_limits<unsigned>::max())
{
    using Mark = Distance_mark<>;
    using Mark_grid = Mark_grid<Mark>;

    Position_set vpos;

    auto radius_accessibility_test = [&](const GridWorld& world, const Position& position, const Mark& mark)
    {
        return mark.distance() <= radius && accessibility_test(world, position, mark);
    };
    auto visit = [&](const GridWorld&, const Position& position, Mark&)
    {
        vpos.insert(position);
    };
    Mark_grid marks;
    spread_from_start(marks, world, start, exploration_rules, radius_accessibility_test, visit);

    return vpos;
}
