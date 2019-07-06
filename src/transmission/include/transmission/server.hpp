// server.hpp

#ifndef RVT_SERVER_HPP
#define RVT_SERVER_HPP

#include <transmission/packet.hpp>

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <functional>
#include <deque>

namespace rvt
{

class server
{
    using callback = std::function<int(server&, const packet&)>;

    public:

    server();

    void load(const std::string &infile);

    int call(const packet& pack);

    std::string get_format(uint16_t id) const;
    std::string get_name(uint16_t id) const;

    std::vector<packet> history;
    std::map<std::string, std::string> context;
    std::map<uint16_t, callback> callbacks;
    std::map<uint16_t, std::string> parse_formats;
    std::map<uint16_t, std::string> message_names;
};

std::ostream& operator << (std::ostream &os, const server &serv);

} // namespace rvt

#endif // RVT_SERVER_HPP
