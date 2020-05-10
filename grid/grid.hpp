#pragma once

#include "grid_view.hpp"
#include "grid_base.hpp"

template <typename Type>
class Grid : public Grid_base
{
    using Base = Grid_base;

public:
    using Value_type = Type;
    using Data_container = std::vector<Value_type>;
    using Reference = typename Data_container::reference;
    using Const_reference = typename Data_container::const_reference;
    using Pointer = typename Data_container::pointer;
    using Const_pointer = typename Data_container::const_pointer;
    using Iterator = Grid_iterator<Grid*>;
    using Const_iterator = Grid_iterator<const Grid*>;
    using View = Grid_view<Grid>;
    using Const_view = Grid_view<const Grid>;
    // standard compliance:
    using value_type = Value_type;
    using const_reference = Value_type;
    using reference = Reference;
    using const_pointer = Const_pointer;
    using pointer = Pointer;
    using iterator = Iterator;
    using const_iterator = Const_iterator;

public:
    Grid()
        : Base()
    {}

    Grid(unsigned width, unsigned height, const Type& value = Type())
        : Base(width, height), data_(width * height, value)
    {}

    Grid(const Dimension& dim, const Type& value = Type())
        : Grid(dim.x, dim.y, value)
    {}

    //TODO Grid(const GT& other)

    inline Const_reference get(int i, int j) const { return data_[j * this->width() + i]; }
    inline Const_reference get(const Position& pos) const { return get(pos.x, pos.y); }
    inline Reference get(int i, int j) { return data_[j * this->width() + i]; }
    inline Reference get(const Position& pos) { return get(pos.x, pos.y); }

    inline Const_iterator begin() const { return Const_iterator(this, Position()); }
    inline Iterator begin() { return Iterator(this, Position()); }
    inline Const_iterator cbegin() const { return begin(); }
    inline Const_iterator end() const{ return Const_iterator(this, Position(0, height())); }
    inline Iterator end() { return Iterator(this, Position(0, height())); }
    inline Const_iterator cend() const { return end(); }

    //TODO function iter_range(const GT& grid) instead
    //     function iter_range(GT&& grid) delete

    inline Const_iterator make_iterator(const Position& position) const { return Const_iterator(this, position); }
    inline Iterator make_iterator(const Position& position) { return Iterator(this, position); }

    inline Const_view view(Offset offset, Dimension dimension) const
    {
        int right = std::min<int>(this->width(), offset.x + dimension.x);
        int bottom = std::min<int>(this->height(), offset.y + dimension.y);
        offset.x = std::max(0, offset.x);
        offset.y = std::max(0, offset.y);
        dimension.x = right - offset.x;
        dimension.y = bottom - offset.y;
        return Const_view(*this, offset, dimension);
    }

    inline View view(Offset offset, Dimension dimension)
    {
        int right = std::min<int>(this->width(), offset.x + dimension.x);
        int bottom = std::min<int>(this->height(), offset.y + dimension.y);
        offset.x = std::max(0, offset.x);
        offset.y = std::max(0, offset.y);
        dimension.x = right - offset.x;
        dimension.y = bottom - offset.y;
        return View(*this, offset, dimension);
    }

    //TODO subgrid()

    inline const Data_container& data() const { return data_; }
    inline Data_container& data() { return data_; }

    void clear()
    {
        this->mutable_dimension() = Dimension();
        data_.clear();
    }

    void resize(unsigned width, unsigned height, const Type& value = Type())
    {
        if (this->width() == width)
            data_.resize(width * height, value);
        else
        {
            Data_container data(width * height, value);
            for (unsigned j = 0, endj = std::min(this->height(), height); j < endj; ++j)
                for (unsigned i = 0, endi = std::min(this->width(), width); i < endi; ++i)
                    data[j * width + i] = std::move(get(i,j));
            data_ = std::move(data);
        }
        this->mutable_dimension() = Dimension(width, height);
    }

    void resize(const Dimension& dimension, const Type& value = Type())
    {
        resize(dimension.x, dimension.y, value);
    }

    void swap(Grid& other)
    {
        Base::swap(other);
        std::swap(data_, other.data_);
    }

    inline bool operator==(const Grid& rhs) const
    {
        return static_cast<const Base&>(*this) == rhs && data_ == rhs.data_;
    }

    inline bool operator!=(const Grid& rhs) const { return !(*this == rhs); }

private:
    std::vector<Type> data_;
};
