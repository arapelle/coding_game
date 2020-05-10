#pragma once

#include "spread_exploration.hpp"

/**
 * direction_to(grid, start, dest, accessibility_test, stop_condition) -> Direction
 */
template <typename GridWorld, typename DirectionExplorationRules, typename AccessibilityTest,
          typename StopCondition = Default>
typename DirectionExplorationRules::Direction
direction_to(const GridWorld& world, const Position& start, const Position& destination,
             DirectionExplorationRules exploration_rules,
             AccessibilityTest accessibility_test,
             StopCondition stop_cond = StopCondition())
{
    auto stop_condition = resolve_default<No_stop_condition>(std::move(stop_cond));
    using Direction = typename DirectionExplorationRules::Direction;
    using Mark = Forward_step_mark<Direction>;
    using Mark_grid = Mark_grid<Mark>;

    auto destination_reached_stop_condition = [&](const Mark_grid& marks)
    {
        return marks.is_treated(marks.get(destination)) || stop_condition(marks);
    };
    Mark_grid marks;
    spread_from_start(marks, world, start, exploration_rules,
                      accessibility_test, default_square_visitor, destination_reached_stop_condition);

    Direction dir_to = Directions4::undefined_direction;
    if (const Mark* pmark = &marks.get(destination); marks.is_treated(*pmark))
    {
        while (pmark->previous_position() != start)
            pmark = &marks.get(pmark->previous_position());
        dir_to = pmark->direction_to_square();
    }
    return dir_to;
}
