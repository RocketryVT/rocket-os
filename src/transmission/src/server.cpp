// server.cpp

#include <transmission/serial.hpp>
#include <transmission/server.hpp>

#include <iomanip>
#include <sstream>
#include <fstream>
#include <iterator>

namespace rvt
{

server::server() :

    history(),
    context{{"sync bytes", "0xAA 0x14"}},
    callbacks
    {
        {0, [] (server &serv, const packet &pack)
        {
            std::vector<uint8_t> data(pack.data());
            std::string key, value;
            data >> key >> value;
            if (key == "" || value == "") return 1;
            serv.context[key] = value;
            return 0;
        }},

        {1, [] (server &serv, const packet &pack)
        {
            std::vector<uint8_t> data(pack.data());
            uint16_t id;
            std::string format;
            data >> id >> format;
            serv.parse_formats[id] = format;
            return 0;
        }},
        
        {2, [] (server &serv, const packet &pack)
        {
            std::vector<uint8_t> data(pack.data());
            uint16_t id;
            std::string name;
            data >> id >> name;
            serv.message_names[id] = name;
            return 0;
        }}
    },
    parse_formats{{0, "s s"},
                  {1, "u16 s"},
                  {2, "u16 s"}},
    message_names{{0, "DEF"},
                  {1, "FMT"},
                  {2, "NAME"}} { }

int server::call(const packet& pack)
{
    packet copy(pack);
    copy.format() = get_format(pack.id());
    copy.name() = get_name(pack.id());
    history.push_back(copy);
    auto iter = callbacks.find(copy.id());
    if (iter != callbacks.end())
    {
        return iter->second(*this, copy);
    }
    return -1;
}

std::string server::get_format(uint16_t id) const
{
    auto iter = parse_formats.find(id);
    if (iter != parse_formats.end())
    {
        return iter->second;
    }
    return "";
}

std::string server::get_name(uint16_t id) const
{
    auto iter = message_names.find(id);
    if (iter != message_names.end())
    {
        return iter->second;
    }
    return "";
}

std::ostream& operator << (std::ostream &os, const server &serv)
{
    std::stringstream ss;
    ss << "< Context >" << std::endl;
    for (auto e : serv.context)
    {
        ss << std::setw(20) << e.first << ": "
           << e.second << std::endl;
    }
    ss << "< Callbacks >" << std::endl;
    for (auto e : serv.callbacks)
    {
        ss << std::hex << std::right << std::setw(12) << e.first << ": "
           << std::left << e.second.target_type().name() << std::endl;
    }
    ss << "< Parse Formats >" << std::endl;
    for (auto e : serv.parse_formats)
    {
        ss << std::right << std::dec << std::setw(12) << e.first << ": "
           << std::left << e.second << std::endl;
    }
    ss << "< Message Names >" << std::endl;
    for (auto e : serv.message_names)
    {
        ss << std::right << std::dec << std::setw(12) << e.first << ": "
           << std::left << e.second << std::endl;
    }
    ss << "< History >" << std::endl;
    for (size_t i = 0; i < serv.history.size(); ++i)
    {
        auto pack = serv.history[i];
        ss << "  " << std::setw(4) << std::setfill('0')
           << std::hex << std::right << i << " "
           << pprintf("%@ %# '%$' %s", pack) << std::endl;
    }
    return os << ss.str();
}

} // namespace rvt
