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

class Basic_mark
{
public:
    explicit Basic_mark(Exploration_square_status status = Exploration_square_status::Unvisited);
    inline explicit Basic_mark(const Position&) : Basic_mark() {}

    inline Exploration_square_status status() const { return status_; }
    inline void set_status(Exploration_square_status status) { status_ = status; }

    template <typename Action>
    void set_visited(const Position&, const Action&, const Basic_mark&)
    {
        set_status(Exploration_square_status::Visited);
    }

private:
    Exploration_square_status status_;
};

template <typename MarkBase = Basic_mark>
class Distance_mark : public MarkBase
{
public:
    inline unsigned distance() const { return this->distance_; }

    explicit Distance_mark(const Position& position = Position())
        : MarkBase(position),
          distance_(std::numeric_limits<unsigned>::max())
    {}

    template <typename Action>
    void set_visited(const Position& neighbour_position, Action&& action, const Distance_mark& nmark)
    {
        this->MarkBase::set_visited(neighbour_position, std::forward<Action>(action), nmark);
        distance_ = nmark.distance_ + 1;
    }

protected:
    unsigned distance_;
};

template <typename Action, typename MarkBase = Basic_mark>
class Action_mark_base : public MarkBase
{
public:
    explicit Action_mark_base(const Position& position = Position())
        : MarkBase(position),
          action_(Directions4::undefined_direction)
    {}

    inline const Action& action() const { return action_; }
    inline Action& action() { return action_; }

    void set_visited(Position neighbour_position, Action&& action, const Action_mark_base& nmark)
    {
        this->MarkBase::set_visited(neighbour_position, static_cast<const Action&>(action), nmark);
        action_ = std::move<Action>(action);
    }

    void set_visited(Position neighbour_position, const Action& action, const Action_mark_base& nmark)
    {
        this->MarkBase::set_visited(neighbour_position, action, nmark);
        action_ = action;
    }

private:
    Action action_;
};

template <typename MarkBase = Basic_mark>
class Link_mark_base : public MarkBase
{
public:
    explicit Link_mark_base(const Position& position = Position())
        : MarkBase(position), link_position_(position)
    {}

    inline const Position& link_position() const { return link_position_; }

    template <class Action>
    void set_visited(Position neighbour_position, Action&& action, const Link_mark_base& nmark)
    {
        this->MarkBase::set_visited(neighbour_position, std::forward<Action>(action), nmark);
        link_position_ = neighbour_position;
    }

private:
    Position link_position_;
};

template <typename Action, typename MarkBase = Basic_mark>
using Step_mark_base = Link_mark_base<Action_mark_base<Action, MarkBase>>;

template <typename Direction, typename MarkBase = Basic_mark>
class Forward_step_mark : public Step_mark_base<Direction, MarkBase>
{
    using Base = Step_mark_base<Direction, MarkBase>;

public:
    explicit Forward_step_mark(const Position& position = Position())
        : Base(position)
    {}

    inline const Position& previous_position() const { return this->link_position(); }
    inline Direction direction_to_square() const { return this->action(); }
    inline Direction direction_to_previous_square() const { return this->action().opposed(); }
};

template <typename Direction, typename MarkBase = Basic_mark>
class Backward_step_mark : public Step_mark_base<Direction, MarkBase>
{
    using Base = Step_mark_base<Direction, MarkBase>;

public:
    explicit Backward_step_mark(const Position& position = Position())
        : Base(position)
    {}

    inline const Position& next_position() const { return this->link_position(); }
    inline Direction direction_to_square() const { return this->action(); }
    inline Direction direction_to_previous_square() const { return this->action().opposed(); }
};

template <typename MarkType>
class Mark_grid : public Grid<MarkType>
{
    using Base = Grid<MarkType>;

public:
    using Mark = typename Base::Value_type;

    using Base::Base;

    void resize(const Dimension& dimension) { this->Base::resize(dimension, Mark()); }

    Exploration_square_status status(const Mark& mark) { return mark.status(); }

    inline Mark make_visited_mark(const Position& position) const { return Mark(position); }

    template <typename Action>
    inline void set_visited(Mark& mark_to_visit, const Position& position, const Mark& mark, Action&& action) const
    {
        mark_to_visit.set_visited(position, std::forward<Action>(action), mark);
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
template <typename ExplorationPolicy, typename MarkGrid, typename GridWorld, typename AccessibilityTest,
          typename SquareVisitor = Default, typename ExplorationRules = Default, typename StopCondition = Default>
void spread_exploration(const ExplorationPolicy&, MarkGrid& marks, GridWorld& world, const Position& first_position,
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

    treat_square(first_position, marks.get(first_position), marks.make_visited_mark(first_position));

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

template <typename MarkGrid, typename GridWorld, typename AccessibilityTest,
          typename SquareVisitor = Default, typename ExplorationRules = Default, typename StopCondition = Default>
void spread_from_start(MarkGrid& marks, GridWorld& world, const Position& start,
                       ExplorationRules exploration_rules,
                       AccessibilityTest accessibility_test,
                       SquareVisitor visitor = SquareVisitor(),
                       StopCondition stop_cond = StopCondition())
{
    spread_exploration(Forward_exploration{}, marks, world, start, exploration_rules, accessibility_test, visitor, stop_cond);
}

template <typename MarkGrid, typename GridWorld, typename AccessibilityTest,
          typename SquareVisitor = Default, typename ExplorationRules = Default, typename StopCondition = Default>
void spread_from_destination(MarkGrid& marks, GridWorld& world, const Position& start,
                             ExplorationRules exploration_rules,
                             AccessibilityTest accessibility_test,
                             SquareVisitor visitor = SquareVisitor(),
                             StopCondition stop_cond = StopCondition())
{
    spread_exploration(Backward_exploration{}, marks, world, start, exploration_rules, accessibility_test, visitor, stop_cond);
}

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

/**
 * direction_to(grid, start, dest, accessibility_test, stop_condition) -> Direction
 */
template <typename GridWorld, typename DirectionExplorationRules, typename AccessibilityTest,
          typename StopCondition = Default>
Direction4 direction_to(const GridWorld& world, const Position& start, const Position& destination,
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

    inline Position next_position(const Position& position, Direction4 dir) const
    {
        Position next_pos = this->ExplorationRulesBase::next_position(position, dir);
        return project_on_torus(next_pos);
    }

    inline Position previous_position(const Position& position, Direction4 dir) const
    {
        Position prev_pos = this->ExplorationRulesBase::previous_position(position, dir);
        return project_on_torus(prev_pos);
    }

    Position& project_on_torus(Position& position) const
    {
        assert(grid_);
        if (!grid_->contains(position))
        {
            unsigned gwidth = grid_->width();
            position.x = (position.x % gwidth + gwidth) % gwidth;
            unsigned gheight = grid_->height();
            position.y %= (position.y % gheight + gheight) % gheight;
        }
        return position;
    }

    inline Position project_on_torus(const Position& pos) const { Position new_pos(pos); return project_on_torus(new_pos); }

private:
    GridType* grid_;
};

template <class GridType>
using Torus_directions4_exploration_rules = Torus<GridType, Directions4_exploration_rules>;

// Accessibility test decorators:

template <typename AccessibilityTestBase>
class Distance_test : public AccessibilityTestBase
{
public:
    //TODO
};

//---------------------------------------------------

void compilation_test();
