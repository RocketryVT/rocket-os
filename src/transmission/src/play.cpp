
#include <transmission/packet.hpp>
#include <transmission/server.hpp>
#include <transmission/util.hpp>

#include <algorithm>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << std::string(argv[0]) << ": "
            << "requires a filename" << std::endl;
        return 1;
    }

    std::string infile(argv[1]);

    auto packets = rvt::fromFile(infile, 0xAA14);

    if (packets.size() == 0)
    {
        std::cerr << "No packets found." << std::endl;
        return 1;
    }

    std::sort(begin(packets), end(packets),
    [] (const rvt::packet& left, const rvt::packet& right)
    {
        return left.time() < right.time();
    });

    rvt::server serv;
    for (auto p : packets)
    {
        serv.call(p);
    }

    using namespace std::chrono;

    auto real_start = system_clock::now();
    auto sim_start = packets[0].time();
    auto sim_duration = packets[packets.size() - 1].time() - sim_start;

    std::cout << "Log begins at " << rvt::dateString(sim_start)
        << ", and spans " << duration<double, std::ratio<60>>
        {sim_duration}.count() << " minutes." << std::endl;

    for (auto p : serv.history)
    {
        auto sim_time = p.time();
        auto real_time = std::chrono::system_clock::now();
        while (real_time - real_start < sim_time - sim_start)
        {
            real_time = std::chrono::system_clock::now();
        }

        std::cout << pprintf(rvt::packet::print_format, p) << std::endl;
    }

    std::cout << "Done." << std::endl;

    return 0;
}
