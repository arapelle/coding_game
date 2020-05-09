#pragma once

#include "direction/direction.hpp"
#include <vector>

class Vec2i
{
public:
    int x = 0;
    int y = 0;

    Vec2i();
    Vec2i(int x, int y);

    Vec2i& operator+=(const Vec2i& rhs);
    Vec2i& operator-=(const Vec2i& rhs);
    Vec2i& operator*=(int scalar);
    Vec2i& operator*=(double scalar);
    Vec2i& operator/=(int scalar);
    Vec2i& operator/=(double scalar);

    friend Vec2i operator+(const Vec2i& lfs, const Vec2i& rhs);
    friend Vec2i operator-(const Vec2i& lfs, const Vec2i& rhs);
    friend Vec2i operator*(const Vec2i& lfs, int scalar);
    friend Vec2i operator*(const Vec2i& lfs, double scalar);
    friend Vec2i operator/(const Vec2i& lfs, int scalar);
    friend Vec2i operator/(const Vec2i& lfs, double scalar);

    bool operator==(const Vec2i& rfs) const;
    bool operator!=(const Vec2i& rfs) const;
    bool operator<(const Vec2i& rfs) const;

    std::vector<Vec2i> circle_area(unsigned radius) const;
    std::vector<Vec2i> square_area(unsigned radius) const;

    friend std::ostream& operator<<(std::ostream& stream, const Vec2i& vec);
    std::string to_string() const;
};
