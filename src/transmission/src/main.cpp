// main.cpp

#include <transmission/util.hpp>
#include <transmission/packet.hpp>
#include <transmission/server.hpp>

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <fstream>
#include <chrono>
#include <deque>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Requires the name of a binary file." << std::endl;
        return 1;
    }

    rvt::server serv;

    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream datestr;
    datestr << std::put_time(std::gmtime(&time), "%Y-%m-%d-%H-%M-%S");
    serv.context["date"] = datestr.str();

    for (auto pack : rvt::fromFile(std::string(argv[1]), 0xAA14))
    {
        serv.call(pack);
    }

    for (auto pack : serv.history)
    {
    //    std::cout << pack << std::endl;
    }

    std::cout << serv << std::endl;

    for (auto pack : serv.history)
    {
        std::cout << rvt::pprintf(rvt::packet::print_format, pack) << std::endl;
    }

    return 0;
}


