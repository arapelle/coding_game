#include "vec2i.hpp"
#include <sstream>
#include <iostream>
#include <cassert>

Vec2i::Vec2i()
{}

Vec2i::Vec2i(int x, int y)
    : x(x), y(y)
{}

Vec2i& Vec2i::operator+=(const Vec2i& rhs)
{
    x += rhs.x;
    y += rhs.y;
    return *this;
}

Vec2i& Vec2i::operator-=(const Vec2i& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    return *this;
}

Vec2i& Vec2i::operator*=(int scalar)
{
    x *= scalar;
    y *= scalar;
    return *this;
}

Vec2i& Vec2i::operator*=(double scalar)
{
    x *= scalar;
    y *= scalar;
    return *this;
}

Vec2i& Vec2i::operator/=(int scalar)
{
    assert(scalar != 0);
    x /= scalar;
    y /= scalar;
    return *this;
}

Vec2i& Vec2i::operator/=(double scalar)
{
    assert(scalar != 0.);
    x /= scalar;
    y /= scalar;
    return *this;
}

Vec2i operator+(const Vec2i& lfs, const Vec2i& rhs)
{
    Vec2i res = lfs;
    res += rhs;
    return res;
}

Vec2i operator-(const Vec2i& lfs, const Vec2i& rhs)
{
    Vec2i res = lfs;
    res -= rhs;
    return res;
}

Vec2i operator*(const Vec2i& lfs, int scalar)
{
    Vec2i res = lfs;
    res *= scalar;
    return res;
}

Vec2i operator*(const Vec2i& lfs, double scalar)
{
    Vec2i res = lfs;
    res *= scalar;
    return res;
}

Vec2i operator/(const Vec2i& lfs, int scalar)
{
    Vec2i res = lfs;
    res /= scalar;
    return res;
}

Vec2i operator/(const Vec2i& lfs, double scalar)
{
    Vec2i res = lfs;
    res /= scalar;
    return res;
}

bool Vec2i::operator==(const Vec2i& rfs) const
{
    return x == rfs.x && y == rfs.y;
}

bool Vec2i::operator!=(const Vec2i& rfs) const
{
    return x != rfs.x && y != rfs.y;
}

bool Vec2i::operator<(const Vec2i& rfs) const
{
    return x < rfs.x || (x == rfs.x && y < rfs.y);
}

std::vector<Vec2i> Vec2i::circle_area(unsigned radius) const
{
    std::vector<Vec2i> vecs;
    for (int j = -radius; j <= static_cast<int>(radius); ++j)
        for (int i = -radius; i <= static_cast<int>(radius); ++i)
            if (abs(i) + abs(j) <= static_cast<int>(radius))
                vecs.push_back(Vec2i(i,j) + *this);
    return vecs;
}

std::vector<Vec2i> Vec2i::square_area(unsigned radius) const
{
    std::vector<Vec2i> vecs;
    for (int j = -radius; j <= static_cast<int>(radius); ++j)
        for (int i = -radius; i <= static_cast<int>(radius); ++i)
            vecs.push_back(Vec2i(i,j) + *this);
    return vecs;
}

std::ostream& operator<<(std::ostream& stream, const Vec2i& vec)
{
    return stream << vec.x << " " << vec.y;
}

std::string Vec2i::to_string() const
{
    std::ostringstream stream;
    stream << *this;
    return stream.str();
}
