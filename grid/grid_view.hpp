#pragma once

#include "grid_base.hpp"

template <class GT>
class Grid_view : public Grid_base
{
    using Base = Grid_base;

public:
    using Value_type = typename GT::Value_type;
    using Const_reference = typename GT::Const_reference;
    using Reference = typename GT::Reference;
    using Const_pointer = typename GT::Const_pointer;
    using Pointer = typename GT::Pointer;
    using Iterator = Grid_iterator<Grid_view*>;
    using Const_iterator = Grid_iterator<const Grid_view*>;
    using Sub_grid = Grid_view;
    using Const_sub_grid = Grid_view<const std::remove_const_t<GT>>;
    // standard compliance:
    using value_type = Value_type;
    using const_reference = Value_type;
    using reference = Reference;
    using const_pointer = Const_pointer;
    using pointer = Pointer;
    using iterator = Iterator;
    using const_iterator = Const_iterator;

public:
    Grid_view(GT& grid, const Offset& offset, const Dimension& dimension)
        : Base(dimension), grid_(&grid), offset_(offset)
    {}

    Grid_view(GT& grid, const Offset& offset)
        : Grid_view(grid, offset, grid.dimension())
    {}

    Grid_view(GT& grid, const Dimension& dimension)
        : Grid_view(grid, Offset(), dimension)
    {}

    explicit Grid_view(GT& grid)
        : Grid_view(grid, Offset())
    {}

    inline Const_reference get(int i, int j) const { return grid_->get(i + offset_.x, j + offset_.y); }
    inline Const_reference get(const Position& pos) const { return get(pos.x, pos.y); }
    inline Reference get(int i, int j) { return grid_->get(i + offset_.x, j + offset_.y); }
    inline Reference get(const Position& pos) { return get(pos.x, pos.y); }

    inline Const_iterator begin() const { return Const_iterator(this, Position()); }
    inline Iterator begin() { return Iterator(this, Position()); }
    inline Const_iterator cbegin() const { return begin(); }
    inline Const_iterator end() const{ return Const_iterator(this, Position(0, height())); }
    inline Iterator end() { return Iterator(this, Position(0, height())); }
    inline Const_iterator cend() const { return end(); }

    inline Const_iterator make_iterator(const Position& position) const { return Const_iterator(this, position); }
    inline Iterator make_iterator(const Position& position) { return Iterator(this, position); }

    inline Const_sub_grid subgrid(Offset offset, Dimension dimension) const
    {
        int right = std::min<int>(this->width(), offset_.x + offset.x + dimension.x);
        int bottom = std::min<int>(this->height(), offset_.y + offset.y + dimension.y);
        offset.x = std::max(0, offset_.x + offset.x);
        offset.y = std::max(0, offset_.y + offset.y);
        dimension.x = right - offset.x;
        dimension.y = bottom - offset.y;
        return Const_sub_grid(*grid_, offset_ + offset, dimension);
    }

    inline Sub_grid subgrid(Offset offset, Dimension dimension)
    {
        int right = std::min<int>(this->width(), offset_.x + offset.x + dimension.x);
        int bottom = std::min<int>(this->height(), offset_.y + offset.y + dimension.y);
        offset.x = std::max(0, offset_.x + offset.x);
        offset.y = std::max(0, offset_.y + offset.y);
        dimension.x = right - offset.x;
        dimension.y = bottom - offset.y;
        return Sub_grid(*grid_, offset_ + offset, dimension);
    }

    //TODO view()
    //TODO resize()
    //TODO move()

private:
    GT* grid_;
    Position offset_;
};
