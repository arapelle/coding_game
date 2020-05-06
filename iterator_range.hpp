#pragma once

#include <algorithm>

template <typename Iter>
class Iterator_iterator
{
public:
    inline explicit Iterator_iterator(Iter iter)
        : iter_(iter)
    {}

    inline Iterator_iterator& operator++()
    {
        ++iter_;
        return *this;
    }

    inline Iterator_iterator operator++(int)
    {
        Iterator_iterator iter(*this);
        ++(*this);
        return iter;
    }

    inline Iterator_iterator& operator--()
    {
        --iter_;
        return *this;
    }

    inline Iterator_iterator operator--(int)
    {
        Iterator_iterator iter(*this);
        --(*this);
        return iter;
    }

    inline const Iter& operator*() const { return iter_; }
    inline Iter& operator*() { return iter_; }

    inline bool operator==(const Iterator_iterator& rhs) const { return iter_ == rhs.iter_; }
    inline bool operator!=(const Iterator_iterator& rhs) const { return !(*this == rhs); }

private:
    Iter iter_;
};

template <typename Iter>
struct Iterator_range
{
    inline explicit Iterator_range(Iter begin, Iter end)
        : first_(Iterator_iterator<Iter>(begin)), end_(Iterator_iterator<Iter>(end))
    {}

    inline const Iterator_iterator<Iter>& begin() const { return first_; }
    inline Iterator_iterator<Iter>& begin() { return first_; }
    inline const Iterator_iterator<Iter>& end() const { return end_; }
    inline Iterator_iterator<Iter>& end() { return end_; }

private:
    Iterator_iterator<Iter> first_;
    Iterator_iterator<Iter> end_;
};

//TODO make_iter_range()
