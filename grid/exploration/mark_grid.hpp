#pragma once

#include "mark.hpp"
#include "grid/grid.hpp"

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
