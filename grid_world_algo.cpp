#include "grid_world_algo.hpp"

Mark::Mark()
    : dir_to_square_(Directions4::undefined_direction), distance_(std::numeric_limits<unsigned>::max())
{}

Mark::Mark(Position prev_pos, Direction4 dir_to_square, unsigned distance)
    : prev_pos_(prev_pos), dir_to_square_(dir_to_square), distance_(distance)
{}

//--------------------------------------------------------------------------------

class Square
{
public:
    explicit Square(bool value = true)
        : value_(value)
    {}

    bool is_free() const { return value_; }

private:
    bool value_;
};

using Grid_world = Grid<Square>;

struct AccessibilityTest
{
    bool operator()(const Grid_world& world, const Position& position, const Mark& )
    {
        const Square& square = world.get(position);
        return square.is_free();
    }
};

void compilation_test()
{
    Grid_world world(8, 6, Square(true));
    AccessibilityTest acc_test;
    Position start(3,2);
    Position dest(6,4);
    spread_from_start(world, start, acc_test);
    reachable_squares(world, start, acc_test, 2);
    reachable_positions(world, start, acc_test, 2);
    direction_to(world, start, dest, acc_test);
}
