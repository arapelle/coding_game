#include "vec2u.hpp"
#include <sstream>
#include <iostream>
#include <cassert>

Vec2u::Vec2u()
{}

Vec2u::Vec2u(int x, int y)
    : x(x), y(y)
{}

Vec2u& Vec2u::operator+=(const Vec2u& rhs)
{
    x += rhs.x;
    y += rhs.y;
    return *this;
}

Vec2u& Vec2u::operator-=(const Vec2u& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    return *this;
}

Vec2u& Vec2u::operator*=(int scalar)
{
    x *= scalar;
    y *= scalar;
    return *this;
}

Vec2u& Vec2u::operator*=(double scalar)
{
    x *= scalar;
    y *= scalar;
    return *this;
}

Vec2u& Vec2u::operator/=(int scalar)
{
    assert(scalar != 0);
    x /= scalar;
    y /= scalar;
    return *this;
}

Vec2u& Vec2u::operator/=(double scalar)
{
    assert(scalar != 0.);
    x /= scalar;
    y /= scalar;
    return *this;
}

Vec2u operator+(const Vec2u& lfs, const Vec2u& rhs)
{
    Vec2u res = lfs;
    res += rhs;
    return res;
}

Vec2u operator-(const Vec2u& lfs, const Vec2u& rhs)
{
    Vec2u res = lfs;
    res -= rhs;
    return res;
}

Vec2u operator*(const Vec2u& lfs, int scalar)
{
    Vec2u res = lfs;
    res *= scalar;
    return res;
}

Vec2u operator*(const Vec2u& lfs, double scalar)
{
    Vec2u res = lfs;
    res *= scalar;
    return res;
}

Vec2u operator/(const Vec2u& lfs, int scalar)
{
    Vec2u res = lfs;
    res /= scalar;
    return res;
}

Vec2u operator/(const Vec2u& lfs, double scalar)
{
    Vec2u res = lfs;
    res /= scalar;
    return res;
}

bool Vec2u::operator==(const Vec2u& rfs) const
{
    return x == rfs.x && y == rfs.y;
}

bool Vec2u::operator!=(const Vec2u& rfs) const
{
    return x != rfs.x && y != rfs.y;
}

bool Vec2u::operator<(const Vec2u& rfs) const
{
    return x < rfs.x || (x == rfs.x && y < rfs.y);
}

std::vector<Vec2u> Vec2u::circle_area(unsigned radius) const
{
    std::vector<Vec2u> vecs;
    for (int j = -radius; j <= static_cast<int>(radius); ++j)
        for (int i = -radius; i <= static_cast<int>(radius); ++i)
            if (abs(i) + abs(j) <= static_cast<int>(radius))
                vecs.push_back(Vec2u(i,j) + *this);
    return vecs;
}

std::vector<Vec2u> Vec2u::square_area(unsigned radius) const
{
    std::vector<Vec2u> vecs;
    for (int j = -radius; j <= static_cast<int>(radius); ++j)
        for (int i = -radius; i <= static_cast<int>(radius); ++i)
            vecs.push_back(Vec2u(i,j) + *this);
    return vecs;
}

std::ostream& operator<<(std::ostream& stream, const Vec2u& vec)
{
    return stream << vec.x << " " << vec.y;
}

std::string Vec2u::to_string() const
{
    std::ostringstream stream;
    stream << *this;
    return stream.str();
}
