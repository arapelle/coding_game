#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <functional>
#include <cassert>

namespace priv
{
class Bad_direction {};
class Undefined_direction {};
}

class Direction4
{
public:
    using IT = int8_t;

private:
    inline static constexpr std::size_t count_ = 4;
    inline static constexpr std::size_t opposed_offset_ = count_ / 2;
    inline static constexpr std::size_t left_offset_ = opposed_offset_ + 1;
    inline static constexpr std::size_t right_offset_ = opposed_offset_ - 1;
    inline static constexpr IT first_valid_value_ = 0;
    inline static constexpr IT bad_value_ = first_valid_value_ - 1;
    inline static constexpr IT undefined_value_ = bad_value_ - 1;

public:
    inline constexpr bool is_valid() const { return value_ >= first_valid_value_; }
    inline constexpr bool is_bad() const { return value_ == bad_value_; }
    inline constexpr bool is_undefined() const { return value_ == undefined_value_; }
    inline constexpr bool is_defined() const { return value_ > undefined_value_; }

    inline constexpr bool operator==(const Direction4& rhs) const { return value_ == rhs.value_; }
    inline constexpr bool operator<(const Direction4& rhs) const { return value_ < rhs.value_; }
    inline constexpr bool operator!=(const Direction4& rhs) const { return value_ != rhs.value_; }

    inline constexpr Direction4 opposed() const { assert(is_valid()); return Direction4((value_ + opposed_offset_) % count_); }
    inline constexpr Direction4 left() const { assert(is_valid()); return Direction4((value_ + left_offset_) % count_); }
    inline constexpr Direction4 right() const { assert(is_valid()); return Direction4((value_ + right_offset_) % count_); }

    inline constexpr IT index() const { return value_; }
    inline constexpr operator IT() const { return value_; }

private:
    inline constexpr explicit Direction4(IT value) : value_(value) {}
    inline constexpr explicit Direction4(priv::Bad_direction) : value_(bad_value_) {}
    inline constexpr explicit Direction4(priv::Undefined_direction) : value_(undefined_value_) {}

private:
    friend class Directions4;

    IT value_;
};

namespace std
{
template<>
struct hash<Direction4>
{
    std::size_t operator()(const Direction4& dir) const noexcept
    {
        return static_cast<std::size_t>(dir.index());
    }
};
}

class Directions4
{
public:
    inline static constexpr std::size_t count = Direction4::count_;
    inline static constexpr Direction4 north = Direction4(0);
    inline static constexpr Direction4 east = Direction4(1);
    inline static constexpr Direction4 south = Direction4(2);
    inline static constexpr Direction4 west = Direction4(3);
    inline static constexpr Direction4 bad_direction = Direction4(priv::Bad_direction{});
    inline static constexpr Direction4 undefined_direction = Direction4(priv::Undefined_direction{});

    inline static constexpr std::array<Direction4, count> directions{ north, east, south, west };
    inline static constexpr std::array<Direction4, count> opposed_directions{ south, west, north, east };

    inline static constexpr std::array<Direction4, count - 1> other_directions(const Direction4& dir)
    {
        return std::array<Direction4, count - 1>{ dir.opposed(), dir.left(), dir.right() };
    }

    inline static constexpr Direction4 nth(std::size_t index) { assert(index < count); return directions[index]; }

    inline static char to_char(Direction4 dir)
    {
        static std::string_view chars("?%NESW");
        return chars.at(dir.index() + 2);
    }

    inline static std::string_view to_string_view(Direction4 dir)
    {
        static std::array<std::string_view, count + 2> sv_array{ "Undefined", "Bad", "North", "East", "South", "West" };
        return sv_array.at(dir.index() + 2);
    }
};
