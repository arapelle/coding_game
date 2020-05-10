#pragma once

#include "mark.hpp"
#include "mark_grid.hpp"
#include "grid/grid_algo.hpp"
#include "direction/neighbourhood.hpp"
#include "core/default_arg.hpp"
#include <queue>
#include <vector>
#include <type_traits>

struct Dummy_square_visitor
{
    template <typename GridWorld, typename Mark>
    inline constexpr void operator()(GridWorld&, const Position&, Mark&) const {}
};

struct No_stop_condition
{
    template <typename MarkGrid>
    inline constexpr bool operator()(const MarkGrid&) const { return false; }
};

struct Forward_exploration {};

struct Backward_exploration {};

inline constexpr Default default_square_visitor = default_arg;
inline constexpr Default default_stop_condition = default_arg;
/**
 * MarkGrid:
 *   void resize(const Dimension& dimension);
 *   Mark make_visited_mark(const Position& position) const;
 *   Mark make_visited_mark(Position neighbour_position, const Mark& neighbour_mark, Direction4 move_direction) const;
 *   bool is_treated(const Mark& mark) const;
 *   void set_treated(Mark& mark);
 * ExplorationRules:
 *   MoveActions move_actions(const GridWorld& world, const Position& position, const Mark& prev_mark);
 *   Position neighbour(const Position& pos, const MoveAction& move_action_to_neighbour_pos);
 * AccessibilityTest:
 *   bool operator()(const GridWorld& world, const Position& position, const Mark& mark);
 * SquareVisitor:
 *   void operator()(GridWorld& world, const Position& position, Mark& mark);
 * StopCondition:
 *   void operator()(const MarkGrid& marks);
 */
template <typename ExplorationPolicy, typename MarkGrid, typename GridWorld, typename RootPositionIterator,
          typename AccessibilityTest, typename SquareVisitor = Default, typename ExplorationRules = Default,
          typename StopCondition = Default>
void spread_exploration(const ExplorationPolicy&, MarkGrid& marks, GridWorld& world,
                        RootPositionIterator root_position_first, RootPositionIterator root_position_last,
                        ExplorationRules exploration_rules,
                        AccessibilityTest accessibility_test,
                        SquareVisitor visitor = SquareVisitor(),
                        StopCondition stop_cond = StopCondition())
{
    auto visit = resolve_default<Dummy_square_visitor>(std::move(visitor));
    auto stop_condition = resolve_default<No_stop_condition>(std::move(stop_cond));
    using Mark = typename MarkGrid::Value_type;

    marks.resize(world.dimension());
    std::queue<Position> position_queue;

    auto treat_square = [&](const Position& pos, Mark& mark_to_update, Mark&& mark)
    {
        position_queue.push(pos);
        mark_to_update = std::move(mark);
        visit(world, pos, mark_to_update);
    };

    for (; root_position_first != root_position_last; ++root_position_first)
    {
        Position root_position = *root_position_first;
        treat_square(root_position, marks.get(root_position), marks.make_visited_mark(root_position));
    }

    while (!position_queue.empty())
    {
        Position cpos = position_queue.front();
        position_queue.pop();

        if (Mark& cmark = marks.get(cpos); !marks.is_treated(cmark))
        {
            marks.set_treated(cmark);
            if (stop_condition(static_cast<const MarkGrid&>(marks)))
                break;

            for (auto&& action : exploration_rules.move_actions(world, cpos, cmark))
            {
                Position npos;
                if constexpr (std::is_same_v<ExplorationPolicy, Forward_exploration>)
                    npos = exploration_rules.next_position(cpos, action);
                else
                    npos = exploration_rules.previous_position(cpos, action);
                if (world.contains(npos))
                    if (Mark& nmark = marks.get(npos); !marks.is_treated(nmark))
                    {
                        Mark mark = nmark;
                        marks.set_visited(nmark, cpos, cmark, std::forward<decltype(action)>(action));
                        if (accessibility_test(world, npos, mark))
                            treat_square(npos, nmark, std::move(mark));
                    }
            }
        }
    }
}

// Spread from start positions:

template <typename MarkGrid, typename GridWorld, typename RootPositionIterator, typename AccessibilityTest,
          typename SquareVisitor = Default, typename ExplorationRules = Default, typename StopCondition = Default>
inline void spread_from_starts(MarkGrid& marks, GridWorld& world,
                               RootPositionIterator root_position_first, RootPositionIterator root_position_last,
                               ExplorationRules exploration_rules,
                               AccessibilityTest accessibility_test,
                               SquareVisitor visitor = SquareVisitor(),
                               StopCondition stop_cond = StopCondition())
{
    spread_exploration(Forward_exploration{}, marks, world, root_position_first, root_position_last,
                       exploration_rules, accessibility_test, visitor, stop_cond);
}

template <typename MarkGrid, typename GridWorld, typename AccessibilityTest,
          typename SquareVisitor = Default, typename ExplorationRules = Default, typename StopCondition = Default>
inline void spread_from_start(MarkGrid& marks, GridWorld& world, const Position& start,
                              ExplorationRules exploration_rules,
                              AccessibilityTest accessibility_test,
                              SquareVisitor visitor = SquareVisitor(),
                              StopCondition stop_cond = StopCondition())
{
    spread_from_starts(marks, world, &start, std::next(&start), exploration_rules, accessibility_test, visitor, stop_cond);
}

// Spread from destination positions:

template <typename MarkGrid, typename GridWorld, typename RootPositionIterator, typename AccessibilityTest,
          typename SquareVisitor = Default, typename ExplorationRules = Default, typename StopCondition = Default>
inline void spread_from_destinations(MarkGrid& marks, GridWorld& world,
                                     RootPositionIterator root_position_first, RootPositionIterator root_position_last,
                                     ExplorationRules exploration_rules,
                                     AccessibilityTest accessibility_test,
                                     SquareVisitor visitor = SquareVisitor(),
                                     StopCondition stop_cond = StopCondition())
{
    spread_exploration(Backward_exploration{}, marks, world, root_position_first, root_position_last,
                       exploration_rules, accessibility_test, visitor, stop_cond);
}

template <typename MarkGrid, typename GridWorld, typename AccessibilityTest,
          typename SquareVisitor = Default, typename ExplorationRules = Default, typename StopCondition = Default>
inline void spread_from_destination(MarkGrid& marks, GridWorld& world, const Position& start,
                                    ExplorationRules exploration_rules,
                                    AccessibilityTest accessibility_test,
                                    SquareVisitor visitor = SquareVisitor(),
                                    StopCondition stop_cond = StopCondition())
{
    spread_from_destinations(marks, world, &start, std::next(&start), exploration_rules, accessibility_test, visitor, stop_cond);
}

// Accessibility test decorators:

template <typename AccessibilityTestBase>
class Distance_test : public AccessibilityTestBase
{
public:
    //TODO
};

//---------------------------------------------------

void compilation_test();
