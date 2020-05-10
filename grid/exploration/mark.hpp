#pragma once

#include "grid/grid_types.hpp"
#include <cstdint>

enum class Exploration_square_status : int8_t
{
    Unvisited,
    Visited,
    Treated
};

class Basic_mark
{
public:
    explicit Basic_mark(Exploration_square_status status = Exploration_square_status::Unvisited) : status_(status) {}
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
class Action_mark : public MarkBase
{
public:
    explicit Action_mark(const Position& position = Position())
        : MarkBase(position),
          action_(Directions4::undefined_direction)
    {}

    inline const Action& action() const { return action_; }
    inline Action& action() { return action_; }

    void set_visited(Position neighbour_position, Action&& action, const Action_mark& nmark)
    {
        this->MarkBase::set_visited(neighbour_position, static_cast<const Action&>(action), nmark);
        action_ = std::move<Action>(action);
    }

    void set_visited(Position neighbour_position, const Action& action, const Action_mark& nmark)
    {
        this->MarkBase::set_visited(neighbour_position, action, nmark);
        action_ = action;
    }

private:
    Action action_;
};

template <typename MarkBase = Basic_mark>
class Link_mark : public MarkBase
{
public:
    explicit Link_mark(const Position& position = Position())
        : MarkBase(position), link_position_(position)
    {}

    inline const Position& link_position() const { return link_position_; }

    template <class Action>
    void set_visited(Position neighbour_position, Action&& action, const Link_mark& nmark)
    {
        this->MarkBase::set_visited(neighbour_position, std::forward<Action>(action), nmark);
        link_position_ = neighbour_position;
    }

private:
    Position link_position_;
};

template <typename Action, typename MarkBase = Basic_mark>
using Step_mark = Link_mark<Action_mark<Action, MarkBase>>;

template <typename Direction, typename MarkBase = Basic_mark>
class Forward_step_mark : public Step_mark<Direction, MarkBase>
{
    using Base = Step_mark<Direction, MarkBase>;

public:
    explicit Forward_step_mark(const Position& position = Position())
        : Base(position)
    {}

    inline const Position& previous_position() const { return this->link_position(); }
    inline Direction direction_to_square() const { return this->action(); }
    inline Direction direction_to_previous_square() const { return this->action().opposed(); }
};

template <typename Direction, typename MarkBase = Basic_mark>
class Backward_step_mark : public Step_mark<Direction, MarkBase>
{
    using Base = Step_mark<Direction, MarkBase>;

public:
    explicit Backward_step_mark(const Position& position = Position())
        : Base(position)
    {}

    inline const Position& next_position() const { return this->link_position(); }
    inline Direction direction_to_square() const { return this->action(); }
    inline Direction direction_to_previous_square() const { return this->action().opposed(); }
};
