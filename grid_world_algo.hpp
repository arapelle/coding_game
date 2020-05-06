#pragma once

#include "grid.hpp"
#include "grid_algo.hpp"
#include "neighbourhood.hpp"
#include <queue>
#include <vector>
#include <type_traits>

class Mark
{
public:
    Mark(Position prev_pos, Direction4 dir_to_square, unsigned distance);
    inline const Position& previous_position() const { return prev_pos_; }
    inline Direction4 direction_to_square() const { return dir_to_square_; }
    inline Direction4 direction_to_previous_square() const { return dir_to_square_.opposed(); }
    inline unsigned distance() const { return distance_; }
    inline bool is_valid() const { return dir_to_square_.is_defined(); }

    inline static Mark invalid_mark() { return Mark(); }

private:
    Mark();

private:
    Position prev_pos_;
    Direction4 dir_to_square_;
    unsigned distance_;
};

struct Dummy_square_visitor
{
    template <class... Args>
    inline constexpr void operator()(Args&&...) const {}
};

struct No_stop_condition
{
    template <class... Args>
    inline constexpr bool operator()(Args&&...) const { return false; }
};

/**
 * AccessibilityTest: bool operator()(const GridWorld& world, const Position& position, const Mark& mark)
 * SquareVisitor: void operator()(const GridWorld& world, const Position& position, Mark& mark)
 * StopCondition: void operator()(const Grid<Mark>& marks)
 */
template <typename GridWorld, typename AccessibilityTest,
          typename SquareVisitor = Dummy_square_visitor, typename StopCondition = No_stop_condition>
Grid<Mark> spread_from_start(const GridWorld& world, const Position& start,
                             AccessibilityTest accessibility_test,
                             [[maybe_unused]] SquareVisitor visit = SquareVisitor(),
                             [[maybe_unused]] StopCondition stop_condition = StopCondition())
{
    Grid<Mark> marks(world.width(), world.height(), Mark::invalid_mark());
    std::queue<Position> position_queue;

    Mark mark(start, Directions4::bad_direction, 0);
    if (accessibility_test(world, start, mark))
    {
        Mark& mark_to_update = marks.get(start);
        mark_to_update = mark;
        position_queue.push(start);
        if constexpr (!std::is_same_v<SquareVisitor, Dummy_square_visitor>)
            visit(world, start, mark_to_update);
    }

    while (!position_queue.empty() && !stop_condition(static_cast<const Grid<Mark>&>(marks)))
    {
        Position cpos = position_queue.front();
        position_queue.pop();
        unsigned dist = marks.get(cpos).distance() + 1;

        for (Direction4 dir : Directions4::directions)
        {
            Position npos = neighbour(cpos, dir);
            Mark mark(cpos, dir, dist);
            if (world.contains(npos) && !marks.get(npos).is_valid() && accessibility_test(world, npos, mark))
            {
                Mark& mark_to_update = marks.get(npos);
                mark_to_update = mark;
                position_queue.push(npos);
                if constexpr (!std::is_same_v<SquareVisitor, Dummy_square_visitor>)
                    visit(world, start, mark_to_update);
            }
        }
    }

    return marks;
}

/**
 * reachable_squares(grid, pos, accessibility_test, radius) -> vector<Const_iterator>
 */
template <typename GridWorld, typename AccessibilityTest>
std::vector<typename GridWorld::Const_iterator>
reachable_squares(const GridWorld& world, const Position& start, AccessibilityTest accessibility_test,
                  unsigned radius = std::numeric_limits<unsigned>::max())
{
    std::vector<typename GridWorld::Const_iterator> viter;

    auto radius_accessibility_test = [&](const GridWorld& world, const Position& position, const Mark& mark)
    {
        return mark.distance() <= radius && accessibility_test(world, position, mark);
    };
    auto visit = [&](const GridWorld& world, const Position& position, Mark&)
    {
        viter.push_back(world.make_iterator(position));
    };
    spread_from_start(world, start, radius_accessibility_test, visit);

    return viter;
}

/**
 * reachable_positions(grid, pos, accessibility_test, radius) -> vector<Position>
 */
template <typename GridWorld, typename AccessibilityTest>
Position_set
reachable_positions(const GridWorld& world, const Position& start, AccessibilityTest accessibility_test,
                    unsigned radius = std::numeric_limits<unsigned>::max())
{
    Position_set vpos;

    auto radius_accessibility_test = [&](const GridWorld& world, const Position& position, const Mark& mark)
    {
        return mark.distance() <= radius && accessibility_test(world, position, mark);
    };
    auto visit = [&](const GridWorld&, const Position& position, Mark&)
    {
        vpos.insert(position);
    };
    spread_from_start(world, start, radius_accessibility_test, visit);

    return vpos;
}

/**
 * direction_to(grid, start, dest, accessibility_test, stop_condition) -> Direction
 */
template <typename GridWorld, typename AccessibilityTest, typename StopCondition = No_stop_condition>
Direction4 direction_to(const GridWorld& world, const Position& start, const Position& destination,
                         AccessibilityTest accessibility_test,
                         StopCondition stop_condition = StopCondition())
{
    auto destination_reached_stop_condition = [&](const Grid<Mark>& marks)
    {
        return marks.get(destination).is_valid() || stop_condition(marks);
    };
    Grid<Mark> marks = spread_from_start(world, start, accessibility_test, Dummy_square_visitor(), destination_reached_stop_condition);

    Direction4 dir_to = Directions4::undefined_direction;
    if (const Mark* pmark = &marks.get(destination); pmark->is_valid())
    {
        while (pmark->previous_position() != start)
            pmark = &marks.get(pmark->previous_position());
        dir_to = pmark->direction_to_square();
    }
    return dir_to;
}
