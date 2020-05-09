TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        core/algo.cpp \
        direction/direction.cpp \
        grid/exploration/spreading_exploration.cpp \
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
    grid/exploration/spreading_exploration.hpp \
    grid/grid.hpp \
    grid/grid_algo.hpp \
    grid/grid_base.hpp \
    grid/grid_iterator.hpp \
    grid/grid_types.hpp \
    grid/grid_view.hpp \
    grid/iterator_range.hpp \
    grid/neighbourhood.hpp \
    vec2/vec2i.hpp \
    vec2/vec2u.hpp
