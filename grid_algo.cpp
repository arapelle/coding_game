#include "grid_algo.hpp"

bool includes(const Position_set& includer, const Position_set& included)
{
    return std::includes(includer.begin(), includer.end(), included.begin(), included.end());
}

Position_set set_difference(const Position_set& lhs, const Position_set& rhs)
{
    Position_set res;
    std::set_difference(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(), std::inserter(res, res.end()));
    return res;
}

Position_set set_intersection(const Position_set& lhs, const Position_set& rhs)
{
    Position_set res;
    std::set_intersection(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(), std::inserter(res, res.end()));
    return res;
}

Position_set set_symmetric_difference(const Position_set& lhs, const Position_set& rhs)
{
    Position_set res;
    std::set_symmetric_difference(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(), std::inserter(res, res.end()));
    return res;
}

Position_set set_union(const Position_set& lhs, const Position_set& rhs)
{
    Position_set res;
    std::set_union(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(), std::inserter(res, res.end()));
    return res;
}
