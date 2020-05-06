#include "grid_world_algo.hpp"
#include "grid.hpp"

#include <iostream>
#include <cstdlib>

//grid_world API: (grid_world_algo.hpp)
/*
.Grid<Mark>& iparam
.Exploration
.spread_from_start -> spread_from_one_position
  .Mark created by Navigation
  .accessibility_test(world, from_pos, to_pos, mark)
(.CostEval)
(.Heuristic)
*/

//TODO spread_from_dest(grid, dest, is_reachable, stop_condition) -> Grid<Mark>

//TODO command.hpp: class Command; class Command_vector : vector<unique_ptr<Command>>; string_to_commands(string) -> Command_vector

//TODO strategy multi

void test_grid()
{
    Grid<int> gdata(3,2, 999);
    for (auto iter = gdata.begin(), end_iter = gdata.end(); iter != end_iter; ++iter)
    {
        std::cout << "[" << iter.position() << "]: " << *iter << ", ";
        if (iter.position().x == 0)
            std::cout << '\n';
    }
}

int main()
{
    std::cout << "main" << std::endl;
    test_grid();
    return EXIT_SUCCESS;
}
