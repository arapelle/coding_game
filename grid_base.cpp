#include "grid.hpp"

Grid_base::Grid_base(unsigned width, unsigned height)
    : dim_(width, height)
{}

Grid_base::Grid_base(const Dimension& dimension)
    : dim_(dimension)
{}

void Grid_base::advance(Position& position) const
{
    auto div = std::div(++position.x, width());
    position.x = div.rem;
    position.y += div.quot;
}

void Grid_base::advance_backward(Position& position) const
{
    if (position.x == 0)
    {
        position.x = width();
        --position.y;
    };
    --position.x;
}
