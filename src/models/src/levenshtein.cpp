
#include <levenshtein.hpp>
#include <string>
#include <iostream>

namespace lambda
{

size_t levenshtein(const std::string &left, const std::string &right)
{
    const std::vector<char> lvec(begin(left), end(left)),
                            rvec(begin(right), end(right));
    return levenshtein(lvec, rvec);
}

} // namespace lambda
