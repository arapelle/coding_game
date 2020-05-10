TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        core/algo.cpp \
        direction/direction.cpp \
        grid/exploration/spread_exploration.cpp \
        grid/grid_algo.cpp \
        grid/grid_base.cpp \
        main.cpp \
        vec2/vec2i.cpp \
        vec2/vec2u.cpp

HEADERS += \
    core/algo.hpp \
    core/default_arg.hpp \
    direction/direction.hpp \
    direction/neighbourhood.hpp \
    grid/exploration/direction_to.hpp \
    grid/exploration/exploration_rules/decorator/torus.hpp \
    grid/exploration/exploration_rules/directions4_exploration_rules.hpp \
    grid/exploration/exploration_rules/torus_directions4_exploration_rules.hpp \
    grid/exploration/mark.hpp \
    grid/exploration/mark_grid.hpp \
    grid/exploration/reachable_positions.hpp \
    grid/exploration/reachable_squares.hpp \
    grid/exploration/spread_exploration.hpp \
    grid/grid.hpp \
    grid/grid_algo.hpp \
    grid/grid_base.hpp \
    grid/grid_iterator.hpp \
    grid/grid_types.hpp \
    grid/grid_view.hpp \
    grid/iterator_range.hpp \
    vec2/vec2i.hpp \
    vec2/vec2u.hpp
