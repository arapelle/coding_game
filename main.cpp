#include "grid/exploration/spread_exploration.hpp"

#include <iostream>
#include <cstdlib>

//exploration API: (spread_exploration.hpp)
/*
(.CostEval)
(.Heuristic)
*/

//TODO command.hpp: class Command; class Command_vector : vector<unique_ptr<Command>>; string_to_commands(string) -> Command_vector

//TODO strategy multi

void test_grid()
{
    Grid<int> gdata(3,2, 999);
    for (auto iter = gdata.begin(), end_iter = gdata.end(); iter != end_iter; ++iter)
    {
        std::cout << "[" << iter.position() << "]: " << *iter << ", ";
        if (iter.position().x == (signed)gdata.width() - 1)
            std::cout << '\n';
    }
}

int main()
{
    std::cout << "main" << std::endl;
    test_grid();
    compilation_test();
    return EXIT_SUCCESS;
}
