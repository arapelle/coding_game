TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        algo.cpp \
        direction.cpp \
        grid_algo.cpp \
        grid_base.cpp \
        grid_world_algo.cpp \
        main.cpp \
        vec2i.cpp \
        vec2u.cpp

HEADERS += \
    algo.hpp \
    default_arg.hpp \
    direction.hpp \
    grid.hpp \
    grid_algo.hpp \
    grid_base.hpp \
    grid_iterator.hpp \
    grid_types.hpp \
    grid_view.hpp \
    grid_world_algo.hpp \
    iterator_range.hpp \
    neighbourhood.hpp \
    vec2i.hpp \
    vec2u.hpp
