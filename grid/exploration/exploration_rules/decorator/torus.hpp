#pragma once

#include "grid/grid_types.hpp"

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
