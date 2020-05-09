#pragma once

#include "grid/grid.hpp"
#include "grid/grid_algo.hpp"
#include "direction/neighbourhood.hpp"
#include "core/default_arg.hpp"
#include <queue>
#include <vector>
#include <type_traits>
#include <iostream>

enum class Exploration_square_status : int8_t
{
    Unvisited,
    Visited,
    Treated
};

class Mark_base
{
public:
    inline explicit Mark_base(Exploration_square_status status = Exploration_square_status::Unvisited) : status_(status) {}
    inline Exploration_square_status status() const { return status_; }
    inline void set_status(Exploration_square_status status) { status_ = status; }

private:
    Exploration_square_status status_;
};

class Directions4_mark_base : public Mark_base
{
protected:
    Directions4_mark_base();
    Directions4_mark_base(Exploration_square_status status, Position neighbour_position, Direction4 move_direction, unsigned distance);

    Position neighbour_position_;
    Direction4 move_direction_;
    unsigned distance_;
};

class Directions4_mark : public Directions4_mark_base
{
public:
    Directions4_mark();
    Directions4_mark(Exploration_square_status status, Position neighbour_position, Direction4 move_direction, unsigned distance);
    inline const Position& previous_position() const { return neighbour_position_; }
    inline Direction4 direction_to_square() const { return move_direction_; }
    inline Direction4 direction_to_previous_square() const { return move_direction_.opposed(); }
    inline unsigned distance() const { return distance_; }
};

template <typename MarkType>
class Mark_grid : public Grid<MarkType>
{
    using Base = Grid<MarkType>;

public:
    using Mark = typename Base::Value_type;

    using Base::Base;

    void resize(const Dimension& dimension)
    {
        Mark mark;
        mark.set_status(Exploration_square_status::Unvisited);
        this->Base::resize(dimension, mark);
    }

    Exploration_square_status status(const Mark& mark) { return mark.status(); }

    inline Mark make_visited_mark(const Position& position) const
    {
        return Mark(Exploration_square_status::Visited, position, Directions4::bad_direction, 0);
    }

    inline Mark make_visited_mark(Position neighbour_position, const Mark& neighbour_mark, Direction4 move_direction) const
    {
        return Mark(Exploration_square_status::Visited, neighbour_position, move_direction, neighbour_mark.distance() + 1);
    }

    bool is_treated(const Mark& mark) const { return mark.status() == Exploration_square_status::Treated; }

    void set_treated(Mark& mark) { mark.set_status(Exploration_square_status::Treated); }
};

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

struct Directions4_exploration_rules
{
    template <typename GridWorld, typename Mark>
    inline constexpr const auto& move_actions(const GridWorld&, const Position&, const Mark&) const
    {
        return Directions4::directions;
    }

    inline Position neighbour_position(const Position& position, Direction4 dir) const
    {
        return neighbour(position, dir);
    }
};

inline constexpr Default default_square_visitor = default_arg;
inline constexpr Default default_exploration_rules = default_arg;
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
template <typename MarkGrid, typename GridWorld, typename AccessibilityTest,
          typename SquareVisitor = Default, typename ExplorationRules = Default, typename StopCondition = Default>
void spread_from_start(MarkGrid& marks, GridWorld& world, const Position& start,
                       AccessibilityTest accessibility_test,
                       SquareVisitor visitor = SquareVisitor(),
                       ExplorationRules explo_rules = ExplorationRules(),
                       StopCondition stop_cond = StopCondition())
{
    auto visit = resolve_default<Dummy_square_visitor>(std::move(visitor));
    auto exploration_rules = resolve_default<Directions4_exploration_rules>(std::move(explo_rules));
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

    treat_square(start, marks.get(start), marks.make_visited_mark(start));

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
                if (Position npos = exploration_rules.neighbour_position(cpos, action); world.contains(npos))
                    if (Mark& nmark = marks.get(npos); !marks.is_treated(nmark))
                    {
                        Mark mark = marks.make_visited_mark(cpos, cmark, std::forward<decltype(action)>(action));
                        if (accessibility_test(world, npos, mark))
                            treat_square(npos, nmark, std::move(mark));
                    }
        }
    }
}

/**
 * reachable_squares(grid, pos, accessibility_test, exploration_rules, radius) -> vector<Const_iterator>
 */
template <typename GridWorld, typename AccessibilityTest, typename ExplorationRules = Default>
std::vector<Grid_iterator<GridWorld*>>
reachable_squares(GridWorld& world, const Position& start, AccessibilityTest accessibility_test,
                  ExplorationRules explo_rules = ExplorationRules(),
                  unsigned radius = std::numeric_limits<unsigned>::max())
{
    auto exploration_rules = resolve_default<Directions4_exploration_rules>(std::move(explo_rules));
    using Mark_grid = Mark_grid<Directions4_mark>;
    using Mark = Mark_grid::Mark;
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
    spread_from_start(marks, world, start, radius_accessibility_test, visit, exploration_rules);

    return viter;
}

/**
 * reachable_positions(grid, pos, accessibility_test, radius) -> vector<Position>
 */
template <typename GridWorld, typename AccessibilityTest, typename ExplorationRules = Default>
Position_set
reachable_positions(const GridWorld& world, const Position& start, AccessibilityTest accessibility_test,
                    ExplorationRules explo_rules = ExplorationRules(),
                    unsigned radius = std::numeric_limits<unsigned>::max())
{
    auto exploration_rules = resolve_default<Directions4_exploration_rules>(std::move(explo_rules));
    using Mark_grid = Mark_grid<Directions4_mark>;
    using Mark = Mark_grid::Mark;

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
    spread_from_start(marks, world, start, radius_accessibility_test, visit, exploration_rules);

    return vpos;
}

/**
 * direction_to(grid, start, dest, accessibility_test, stop_condition) -> Direction
 */
template <typename GridWorld, typename AccessibilityTest,
          typename ExplorationRules = Default, typename StopCondition = Default>
Direction4 direction_to(const GridWorld& world, const Position& start, const Position& destination,
                        AccessibilityTest accessibility_test,
                        ExplorationRules explo_rules = ExplorationRules(),
                        StopCondition stop_cond = StopCondition())
{
    auto exploration_rules = resolve_default<Directions4_exploration_rules>(std::move(explo_rules));
    auto stop_condition = resolve_default<No_stop_condition>(std::move(stop_cond));
    using Mark_grid = Mark_grid<Directions4_mark>;
    using Mark = Mark_grid::Mark;

    auto destination_reached_stop_condition = [&](const Mark_grid& marks)
    {
        return marks.is_treated(marks.get(destination)) || stop_condition(marks);
    };
    Mark_grid marks;
    spread_from_start(marks, world, start, accessibility_test, default_arg, exploration_rules, destination_reached_stop_condition);

    Direction4 dir_to = Directions4::undefined_direction;
    if (const Mark* pmark = &marks.get(destination); marks.is_treated(*pmark))
    {
        while (pmark->previous_position() != start)
            pmark = &marks.get(pmark->previous_position());
        dir_to = pmark->direction_to_square();
    }
    return dir_to;
}

// Exploration rules decorators:

template <class GridType, class ExplorationRulesBase>
class Torus : public ExplorationRulesBase
{
public:
    Torus()
        : grid_(nullptr)
    {}

    explicit Torus(GridType& grid)
        : grid_(&grid)
    {}

    void set_grid(GridType& grid) { grid_ = &grid; }

    Position neighbour_position(const Position& position, Direction4 dir) const
    {
        Position npos = this->ExplorationRulesBase::neighbour_position(position, dir);
        assert(grid_);
        if (!grid_->contains(npos))
        {
            unsigned gwidth = grid_->width();
            npos.x = (npos.x % gwidth + gwidth) % gwidth;
            unsigned gheight = grid_->height();
            npos.y %= (npos.y % gheight + gheight) % gheight;
        }
        return npos;
    }

private:
    GridType* grid_;
};

template <class GridType>
using Torus_directions4_exploration_rules = Torus<GridType, Directions4_exploration_rules>;

// Accessibility test decorators:

template <typename AccessibilityTestBase>
class DistanceTest : public AccessibilityTestBase
{
public:
    //TODO
};

//---------------------------------------------------

void compilation_test();
