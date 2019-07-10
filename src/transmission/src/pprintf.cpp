// pretty.cpp

#include <transmission/util.hpp>
#include <transmission/packet.hpp>
#include <transmission/serial.hpp>

#include <sstream>
#include <iomanip>
#include <iterator>

namespace rvt
{

std::string packet2str(const packet &pack);

std::string pprintf(const std::string& fmt, const packet& pack)
{
    std::string ret(fmt);
    
    auto insert_if_find = [&] (const std::string &escape,
                               const std::string &str)
    {
        size_t pos = ret.find(escape);
        if (pos != std::string::npos)
        {
            ret.erase(pos, escape.length());
            ret.insert(pos, str);
        }
    };

    std::stringstream id, sync, name;
    id << std::right << std::hex << std::setw(4)
        << std::setfill('0') << pack.id();
    sync << std::right << std::hex << std::setw(4)
        << std::setfill('0') << pack.sync_bytes();
    name << std::left << std::setw(9)
        << std::setfill('-') << pack.name();

    auto time_point = std::chrono::system_clock::time_point{} +
                      std::chrono::milliseconds(pack.time());

    std::vector<std::pair<std::string, std::string>> conv
    {
        { "%@", dateString(time_point) },
        { "%!", sync.str() },
        { "%#", id.str() },
        { "%$", name.str() },
        { "%s", packet2str(pack) }
    };

    for (auto &pair : conv)
    {
        insert_if_find(pair.first, pair.second);
    }

    return ret;
}

std::string packet2str(const packet &pack)
{
    using namespace std::chrono;

    std::vector<uint8_t> data(pack.data());
    std::vector<std::string> tokens;
    std::istringstream iss(pack.format());
    std::copy(std::istream_iterator<std::string>(iss),
              std::istream_iterator<std::string>(),
              std::back_inserter(tokens));

    auto since_epoch = milliseconds(pack.time());
    auto sec = duration_cast<seconds>(since_epoch);
    auto ms = duration_cast<milliseconds>(since_epoch - sec);

    std::stringstream result;

    for (auto token : tokens)
    {
        if (token == "s")
        {
            std::string str = "";
            data >> str;
            if (str.size() > 0)
            {
                if (result.tellp() > 0) result << " ";
                result << '"' << str << '"';
            }
        }
        else if (token == "u8" && data.size() >= 1)
        {
            uint8_t num = 0;
            data >> num;
            if (result.tellp() > 0) result << " ";
            result << std::to_string(num) << "u8";
        }
        else if (token == "u16" && data.size() >= 2)
        {
            uint16_t num = 0;
            data >> num;
            if (result.tellp() > 0) result << " ";
            result << std::to_string(num) << "u16";
        }
        else if (token == "u32" && data.size() >= 4)
        {
            uint32_t num = 0;
            data >> num;
            if (result.tellp() > 0) result << " ";
            result << num << "u32";
        }
        else if (token == "u64" && data.size() >= 8)
        {
            uint64_t num = 0;
            data >> num;
            if (result.tellp() > 0) result << " ";
            result << std::to_string(num) << "u64";
        }
        else if (token == "n8" && data.size() >= 1)
        {
            int8_t num = 0;
            data >> num;
            if (result.tellp() > 0) result << " ";
            result << std::to_string(num) << "n8";
        }
        else if (token == "n16" && data.size() >= 2)
        {
            int16_t num = 0;
            data >> num;
            if (result.tellp() > 0) result << " ";
            result << std::to_string(num) << "n16";
        }
        else if (token == "n32" && data.size() >= 4)
        {
            int32_t num = 0;
            data >> num;
            if (result.tellp() > 0) result << " ";
            result << std::to_string(num) << "n32";
        }
        else if (token == "n64" && data.size() >= 8)
        {
            int64_t num = 0;
            data >> num;
            if (result.tellp() > 0) result << " ";
            result << std::to_string(num) << "n64";
        }
        else if (token == "f" && data.size() >= 1)
        {
            float num = 0;
            data >> num;
            if (result.tellp() > 0) result << " ";
            result << std::to_string(num) << "f";
        }
        else if (token == "d" && data.size() >= 1)
        {
            double num = 0;
            data >> num;
            if (result.tellp() > 0) result << " ";
            result << std::to_string(num) << "d";
        }
        else if (token == "0x" && data.size() >= 1)
        {
            uint8_t num = 0;
            data >> num;
            if (result.tellp() > 0) result << " ";
            result << "0x" << std::hex << std::setfill('0')
                << std::setw(2) << std::right << static_cast<int>(num);
        }
    }

    if (data.size() > 0)
    {
        result << "(" << data.size() << " bytes)";
    }

    return result.str();
}

} // namespace rvt

