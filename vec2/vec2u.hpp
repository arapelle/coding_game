#pragma once

#include <ostream>
#include <vector>
#include <string>

class Vec2u
{
public:
    unsigned x = 0;
    unsigned y = 0;

    Vec2u();
    Vec2u(int x, int y);

    Vec2u& operator+=(const Vec2u& rhs);
    Vec2u& operator-=(const Vec2u& rhs);
    Vec2u& operator*=(int scalar);
    Vec2u& operator*=(double scalar);
    Vec2u& operator/=(int scalar);
    Vec2u& operator/=(double scalar);

    friend Vec2u operator+(const Vec2u& lfs, const Vec2u& rhs);
    friend Vec2u operator-(const Vec2u& lfs, const Vec2u& rhs);
    friend Vec2u operator*(const Vec2u& lfs, int scalar);
    friend Vec2u operator*(const Vec2u& lfs, double scalar);
    friend Vec2u operator/(const Vec2u& lfs, int scalar);
    friend Vec2u operator/(const Vec2u& lfs, double scalar);

    bool operator==(const Vec2u& rfs) const;
    bool operator!=(const Vec2u& rfs) const;
    bool operator<(const Vec2u& rfs) const;

    std::vector<Vec2u> circle_area(unsigned radius) const;
    std::vector<Vec2u> square_area(unsigned radius) const;

    friend std::ostream& operator<<(std::ostream& stream, const Vec2u& vec);
    std::string to_string() const;
};
