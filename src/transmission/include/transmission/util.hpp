// utility.hpp

#ifndef RVT_UTILITY_HPP
#define RVT_UTILITY_HPP

#include <string>
#include <vector>
#include <chrono>

namespace rvt
{

bool begins_with(const std::string &str, const std::string &begin);

bool ends_with(const std::string &str, const std::string &ending);

std::string wrap(const std::string &str, size_t num_chars, size_t offset = 0);

std::vector<std::string> split_quoted(const std::string &fstr);

uint16_t xorchecksum(const std::vector<uint8_t> &data);

std::string dateString(const std::chrono::system_clock::time_point& time);

} // namespace rvt

#endif // RVT_UTILITY_HPP

