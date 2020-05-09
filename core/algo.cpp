#include "algo.hpp"

std::vector<std::string_view> split(const std::string& str, char delimiter)
{
    std::vector<std::string_view> strs;
    String_tokenizer tokenizer(str, delimiter);
    while (tokenizer.has_token())
        strs.push_back(tokenizer.next_token());
    return strs;
}
