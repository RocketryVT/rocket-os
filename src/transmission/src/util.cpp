// utility.cpp

#include <transmission/util.hpp>
#include <transmission/serial.hpp>

#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

namespace rvt
{

bool begins_with(const std::string &str, const std::string &begin)
{
    return str.rfind(begin, 0) == 0;
}

bool ends_with(const std::string &str, const std::string &ending)
{
    if (ending.length() > str.length()) return false;
    return str.compare(str.length() - ending.length(), ending.length(), ending) == 0;
}

std::string wrap(const std::string &str, size_t num_chars, size_t offset)
{
    std::string copy(str);
    std::stringstream ss;
    ss << std::right;
    while (copy.length() > num_chars)
    {
        ss << std::setw(num_chars + offset + 1)
           << copy.substr(0, num_chars) + "\n";
        copy = copy.substr(num_chars, copy.length());
    }
    ss << std::setw(offset) << "" << copy;
    return ss.str();
}


std::vector<std::string> split_quoted(const std::string &fstr)
{
    std::vector<std::string> tokens;
    std::string token;
    char quote_char = ' ';
    for (size_t i = 0; i < fstr.length(); ++i)
    {
        char c = fstr[i];
        if (c == ' ' && quote_char != ' ')
        {
            token += c;
        }
        else if (c == ' ' || i == fstr.length() - 1)
        {
            if (((c != quote_char || quote_char == ' ') && c != ' ')
                || i == fstr.length() - 1) token += c;
            if (token.length() > 0) tokens.push_back(token);
            token = "";
        }
        else if ((c == '\'' || c == '"') && quote_char == ' ')
        {
            quote_char = c;
            token += c;
        }
        else if (c == quote_char && c != ' ')
        {
            quote_char = ' ';
            token += c;
        }
        else if (c != ' ')
        {
            token += c;
        }
    }
    return tokens;
}

uint16_t xorchecksum(const std::vector<unsigned char> &data)
{
	uint8_t c0 = 0;
	uint8_t c1 = 0;
	for (size_t i = 0; i < data.size(); i += 2)
		c0 ^= data[i];
	for (size_t i = 1; i < data.size(); i += 2)
		c1 ^= data[i];

    std::vector<uint8_t> cs;
    cs << c0 << c1;
    uint16_t sum = 0;
    cs >> sum;
	return sum;
}



std::string dateString(const std::chrono::system_clock::time_point& time)
{
    using namespace std::chrono;

    auto since_epoch = time.time_since_epoch();
    auto sec = duration_cast<seconds>(since_epoch);
    auto ms = duration_cast<milliseconds>(since_epoch) - sec;

    auto tt = system_clock::to_time_t(time);
    std::stringstream datestr;
    datestr << std::put_time(std::gmtime(&tt), "%Y-%m-%d %H:%M:%S")
        << "." << std::right << std::setw(3)
        << std::setfill('0') << ms.count() << " UTC";
    return datestr.str();
}

} // namespace rvt

