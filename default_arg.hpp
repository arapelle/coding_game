#pragma once

#include <utility>

class Default {};
inline constexpr Default default_arg;

template <class DefaultType, class GivenType>
struct Resolve_default
{
    static GivenType&& invoke(GivenType&& value) { return std::forward<GivenType>(value); }
};

template <class DefaultType>
struct Resolve_default<DefaultType, Default>
{
    static DefaultType invoke(const Default&) { return DefaultType(); }
};

template <class DefaultType, class GivenType>
auto resolve_default(GivenType&& value) -> decltype(auto)
{
    return Resolve_default<DefaultType, GivenType>::invoke(std::forward<GivenType>(value));
}
