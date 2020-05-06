#pragma once

#include <string_view>
#include <string>
#include <vector>
#include <algorithm>

template <class Char = char>
class Basic_string_tokenizer
{
public:
    using Value_type = Char;
    using String_view = std::basic_string_view<Value_type>;
    using String = std::basic_string<Value_type>;

private:
    using String_view_const_iterator = typename String_view::const_iterator;

public:
    explicit Basic_string_tokenizer(String_view strv, const Char& sep = Char(' '))
    : str_(strv), current_(str_.begin()), sep_(sep)
    {}

    explicit Basic_string_tokenizer(const String& str, const Char& sep = Char(' '))
    : str_(str.c_str(), str.length()), current_(str_.begin()), sep_(sep)
    {}

    inline bool has_token() const { return current_ != str_.end(); }

    String_view next_token()
    {
        std::size_t len = 0;
        String_view_const_iterator iter = current_;
        for (; iter != str_.end(); ++iter, ++len)
        {
            if (*iter == sep_)
            {
                ++iter;
                break;
            }
        }
        String_view tok(&*current_, len);
        current_ = std::move(iter);
        return tok;
    }

private:
    String_view str_;
    String_view_const_iterator current_;
    Char sep_;
};

using String_tokenizer = Basic_string_tokenizer<>;

std::vector<std::string_view> split(const std::string& str, char delimiter);

template <class Cont, class Elem>
void erase(Cont& c, const Elem& elem)
{
    c.erase(std::remove(c.begin(), c.end(), elem), c.end());
}

template <class Cont, class Predicate>
void erase_if(Cont& c, Predicate pred)
{
    c.erase(std::remove_if(c.begin(), c.end(), pred), c.end());
}

namespace priv
{
template <class It>
auto to_ptr(It iter, It end) -> decltype(&*iter)
{
    return iter != end ? &*iter : nullptr;
}
}

template <class It, class Elem>
auto find(It begin, It end, const Elem& elem) -> decltype(&*begin)
{
    return priv::to_ptr(std::find(begin, end, elem), end);
}

template <class Cont, class Elem>
auto find(const Cont& cont, const Elem& elem) -> decltype(find(cont.begin(), cont.end(), elem))
{
    return find(cont.begin(), cont.end(), elem);
}

template <class It, class Predicate>
auto find_if(It begin, It end, Predicate pred) -> decltype(&*begin)
{
    return priv::to_ptr(std::find_if(begin, end, pred), end);
}

template <class Cont, class Predicate>
auto find_if(const Cont& cont, Predicate pred) -> decltype(find_if(cont.begin(), cont.end(), pred))
{
    return find_if(cont.begin(), cont.end(), pred);
}
