#pragma once

#include "directions4_exploration_rules.hpp"
#include "decorator/torus.hpp"

template <class GridType>
using Torus_directions4_exploration_rules = Torus<GridType, Directions4_exploration_rules>;
