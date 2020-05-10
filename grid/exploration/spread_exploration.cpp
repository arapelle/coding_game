#include "direction_to.hpp"
#include "reachable_positions.hpp"
#include "reachable_squares.hpp"
#include "spread_exploration.hpp"
#include "exploration_rules/torus_directions4_exploration_rules.hpp"

//--------------------------------------------------------------------------------

using Direction4_mark = Forward_step_mark<Direction4, Distance_mark<>>;

class Square
{
public:
    explicit Square(bool value = true)
        : value_(value)
    {}

    bool is_free() const { return value_; }
    void set_value(bool value) { value_ = value; }

private:
    bool value_;
};

using Grid_world = Grid<Square>;

struct Accessibility_test
{
    template <class Mark>
    bool operator()(const Grid_world& world, const Position& position, const Mark& )
    {
        const Square& square = world.get(position);
        return square.is_free();
    }
};

void compilation_test()
{
    Grid_world world(8, 6, Square(true));
    const Grid_world& cworld = world;
    Accessibility_test acc_test;
    Torus_directions4_exploration_rules<Grid_world> exploration_rules(world);
    Position start(3,2);
    Position dest(6,4);
    Mark_grid<Direction4_mark> marks;
    spread_from_start(marks, world, start, exploration_rules, acc_test);
    reachable_squares(cworld, start, exploration_rules, acc_test, 2);
    reachable_positions(world, start, exploration_rules, acc_test, 2);
    direction_to(world, start, dest, exploration_rules, acc_test);
}
