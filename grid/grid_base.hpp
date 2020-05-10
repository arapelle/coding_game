#pragma once

#include "grid_iterator.hpp"
#include "grid_types.hpp"
#include <vector>
#include <algorithm>
#include <cstdlib>

class Grid_base
{
protected:
    inline Grid_base() {}

    Grid_base(unsigned width, unsigned height);
    explicit Grid_base(const Dimension& dimension);

public:
    inline const Dimension& dimension() const { return dim_; }
    inline unsigned width() const { return dim_.x; }
    inline unsigned height() const { return dim_.y; }

    inline bool empty() const { return dim_ == Dimension(); }

    inline bool contains(int i, int j) const { return i >= 0 && static_cast<unsigned>(i) < dim_.x && j >= 0 && static_cast<unsigned>(j) < dim_.y; }
    inline bool contains(const Position& pos) const { return contains(pos.x, pos.y); }

    void advance(Position& position) const;

    inline Position next(Position position) const
    {
        advance(position);
        return position;
    }

    void advance_backward(Position& position) const;

    inline Position previous(Position position) const
    {
        advance_backward(position);
        return position;
    }

    inline bool operator==(const Grid_base& rhs) const { return dim_ == rhs.dim_; }

protected:
    inline Dimension& mutable_dimension() { return dim_; }
    inline void swap(Grid_base& other) { std::swap(dim_, other.dim_); }

private:
    Dimension dim_;
};
