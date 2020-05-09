#pragma once

#include "direction/direction.hpp"
#include <vector>

template <typename Vec2>
Vec2 neighbour(Vec2 vec, Direction4 dir)
{
    switch (dir)
    {
    case Directions4::north: --vec.y; break;
    case Directions4::south: ++vec.y; break;
    case Directions4::east: --vec.x; break;
    case Directions4::west: ++vec.x; break;
    case Directions4::bad_direction:
    case Directions4::undefined_direction:
    default:
        vec = Vec2(-1,-1);
        break;
    }
    return vec;
}

template <typename Vec2>
std::array<Vec2, 4> neighbours4(Vec2 vec)
{
    std::array<Vec2, 4> neighbours;
    for (Direction4 dir : Directions4::directions)
        neighbours[dir.index()] = neighbour(vec, dir);
    return neighbours;
}
