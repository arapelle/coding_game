#pragma once

#include "grid_types.hpp"
#include <set>
#include <algorithm>
#include <cstdint>

class Position_set : public std::set<Position>
{
public:
    using std::set<Position>::set;
};

bool includes(const Position_set& includer, const Position_set& included);

Position_set set_difference(const Position_set& lhs, const Position_set& rhs);

Position_set set_intersection(const Position_set& lhs, const Position_set& rhs);

Position_set set_symmetric_difference(const Position_set& lhs, const Position_set& rhs);

Position_set set_union(const Position_set& lhs, const Position_set& rhs);

// count:
template <typename GridType, typename ValueType>
std::size_t count(const GridType& grid, const ValueType& value)
{
    return std::count(grid.begin(), grid.end(), value);
}

// count_if:
template <typename GridType, typename Predicate>
std::size_t count_if(const GridType& grid, Predicate pred)
{
    return std::count_if(grid.begin(), grid.end(), std::move(pred));
}

// replace:
template <typename GridType, typename ValueType>
void replace(const GridType& grid, const ValueType& value, const ValueType& new_value)
{
    std::replace(grid.begin(), grid.end(), value, new_value);
}

// replace_if:
template <typename GridType, typename Predicate, typename ValueType>
void replace_if(const GridType& grid, Predicate pred, const ValueType& new_value)
{
    std::replace_if(grid.begin(), grid.end(), std::move(pred), new_value);
}

// for_each:
template <typename GridType, typename Function>
void for_each(const GridType& grid, Function func)
{
    std::for_each(grid.begin(), grid.end(), std::move(func));
}

// for_each_if:
template <typename GridType, typename Predicate, typename Function>
void for_each_if(const GridType& grid, Predicate pred, Function func)
{
    using Value_type = typename GridType::Value_type;
    std::for_each(grid.begin(), grid.end(), [&](const Value_type& val){ if (pred(val)) func(val); });
}

// find_all:
template <typename GridType, typename ValueType>
std::vector<typename GridType::Const_iterator> find_all(const GridType& grid, const ValueType& value)
{
    std::vector<typename GridType::Const_iterator> viter;
    for (auto iter = grid.begin(), end_iter = grid.end(); iter != end_iter; ++iter)
        if (*iter == value)
            viter.push_back(iter);
    return viter;
}

template <typename GridType, typename ValueType>
std::vector<typename GridType::Iterator> find_all(GridType& grid, const ValueType& value)
{
    std::vector<typename GridType::Iterator> viter;
    for (auto iter = grid.begin(), end_iter = grid.end(); iter != end_iter; ++iter)
        if (*iter == value)
            viter.push_back(iter);
    return viter;
}

// find_all_if:
template <typename GridType, typename Predicate>
std::vector<typename GridType::Const_iterator> find_all_if(const GridType& grid, Predicate pred)
{
    std::vector<typename GridType::Const_iterator> viter;
    for (auto iter = grid.begin(), end_iter = grid.end(); iter != end_iter; ++iter)
        if (pred(*iter))
            viter.push_back(iter);
    return viter;
}

template <typename GridType, typename Predicate>
std::vector<typename GridType::Iterator> find_all_if(GridType& grid, Predicate pred)
{
    std::vector<typename GridType::Iterator> viter;
    for (auto iter = grid.begin(), end_iter = grid.end(); iter != end_iter; ++iter)
        if (pred(*iter))
            viter.push_back(iter);
    return viter;
}

// find_all_positions:
template <typename GridType, typename ValueType>
Position_set find_all_positions(GridType& grid, const ValueType& value)
{
    Position_set iters;
    for (auto iter = grid.begin(), end_iter = grid.end(); iter != end_iter; ++iter)
        if (*iter == value)
            iters.insert(iter.position());
    return iters;
}

// find_all_positions_if:
template <typename GridType, typename Predicate>
Position_set find_all_positions_if(GridType& grid, Predicate pred)
{
    Position_set iters;
    for (auto iter = grid.begin(), end_iter = grid.end(); iter != end_iter; ++iter)
        if (pred(*iter))
            iters.insert(iter.position());
    return iters;
}
