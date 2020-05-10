#pragma once

#include "spread_exploration.hpp"

/**
 * reachable_squares(grid, pos, accessibility_test, exploration_rules, radius) -> vector<Const_iterator>
 */
template <typename GridWorld, typename ExplorationRules, typename AccessibilityTest>
std::vector<Grid_iterator<GridWorld*>>
reachable_squares(GridWorld& world, const Position& start, ExplorationRules exploration_rules,
                  AccessibilityTest accessibility_test,
                  unsigned radius = std::numeric_limits<unsigned>::max())
{
    using Mark = Distance_mark<>;
    using Mark_grid = Mark_grid<Mark>;
    using Iterator = Grid_iterator<GridWorld*>;

    std::vector<Iterator> viter;

    auto radius_accessibility_test = [&](const GridWorld& world, const Position& position, const Mark& mark)
    {
        return mark.distance() <= radius && accessibility_test(world, position, mark);
    };
    auto visit = [&](GridWorld& world, const Position& position, Mark&)
    {
        viter.push_back(Iterator(&world, position));
    };
    Mark_grid marks;
    spread_from_start(marks, world, start, exploration_rules, radius_accessibility_test, visit);

    return viter;
}
