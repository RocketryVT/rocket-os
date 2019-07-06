// serial.cpp

#include <transmission/serial.hpp>

#include <iostream>

namespace rvt
{

/// \brief Extracts bytes from a vector and inserts them into
///        a null-terminated string
std::vector<uint8_t>& operator >> (std::vector<uint8_t> &vec, std::string &str)
{
    std::vector<uint8_t> data;
    size_t i = 0;
    for (; i < vec.size() && vec[i] != 0; ++i)
        data.push_back(vec[i]);
    str = std::string(data.begin(), data.end());
    if (vec.size() > i && vec[i] == 0) ++i;
    vec.erase(vec.begin(), vec.begin() + i);
    return vec;
}

} // namespace rvt

