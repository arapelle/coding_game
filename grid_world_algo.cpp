#include "grid_world_algo.hpp"

// Directions4_mark_base:

Directions4_mark_base::Directions4_mark_base(Exploration_square_status status, Position neighbour_position,
                                             Direction4 move_direction, unsigned distance)
    : Mark_base(status), neighbour_position_(neighbour_position), move_direction_(move_direction), distance_(distance)
{}

Directions4_mark_base::Directions4_mark_base()
    : Directions4_mark_base(Exploration_square_status::Unvisited, Position(-1,-1),
                            Directions4::undefined_direction, std::numeric_limits<unsigned>::max())
{}

// Directions4_mark:

Directions4_mark::Directions4_mark()
{}

Directions4_mark::Directions4_mark(Exploration_square_status status, Position neighbour_position,
                                   Direction4 move_direction, unsigned distance)
    : Directions4_mark_base(status, neighbour_position, move_direction, distance)
{
}

//--------------------------------------------------------------------------------

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
    bool operator()(const Grid_world& world, const Position& position, const Directions4_mark& )
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
    Mark_grid<Directions4_mark> marks;
    spread_from_start(marks, world, start, acc_test, default_square_visitor, exploration_rules);
    reachable_squares(cworld, start, acc_test, default_exploration_rules, 2);
    reachable_positions(world, start, acc_test, default_exploration_rules, 2);
    direction_to(world, start, dest, acc_test, default_exploration_rules);
}
