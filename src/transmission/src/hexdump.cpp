// hexdump.cpp

#include <transmission/packet.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>

namespace rvt
{

const std::string underline = "\033[4m";
const std::string reverse = "\033[7m";
const std::string red = "\033[31;40m";
const std::string yellow = "\033[39;40m";
const std::string blue = "\033[34;40m";
const std::string green = "\033[40;40m";
const std::string orange = "\033[202;40m";
const std::string cyan = "\033[36;40m";
const std::string gray = "\033[245;40m";
const std::string clear = "\033[0m";

}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << std::string(argv[0]) << ": "
            << "requires a filename" << std::endl;
        return 1;
    }

    std::ifstream infile(argv[1], std::ios::binary);
    if (!infile)
    {
        std::cerr << std::string(argv[0]) << ": "
            << "failed to open " << std::string(argv[1]) << std::endl;
        return 1;
    }

    std::vector<uint8_t> bytes(
        std::istreambuf_iterator<char>{infile}, {});

    std::vector<size_t> addresses;
    std::vector<rvt::packet> packets;
    auto read_iter(begin(bytes)), ending(end(bytes));
    rvt::packet pack;
    auto last_read_iter = read_iter;
    while (rvt::nextPacket(0xAA14, pack, read_iter, ending))
    {
        size_t address = (read_iter - pack.data().size() - 16) - begin(bytes);
        addresses.push_back(address);
        packets.push_back(pack);
        std::cout << pack << std::endl;
    }

    return 0;


    size_t i = 0;
    size_t packet_num = 0;
    while (i < bytes.size())
    {
        std::cout << std::hex << std::right << std::setfill('0');
        std::cout << std::setw(8) << i << "  ";
        for (size_t j = 0; j < 16; ++j)
        {
            if (i + j < bytes.size())
            {
                auto btwn = [] (size_t x, size_t min, size_t max)
                {
                    return x >= min && x < max;
                };

                auto pack = packets.at(packet_num);
                size_t minaddr = addresses[packet_num];
                size_t maxaddr = minaddr + 16 + pack.data().size();
                size_t addr = i + j;
                if (btwn(addr, minaddr, minaddr + 2)) std::cout << rvt::red;
                if (btwn(addr, minaddr + 2, minaddr + 10)) std::cout << rvt::gray;
                if (btwn(addr, minaddr + 10, minaddr + 12)) std::cout << rvt::blue;
                if (btwn(addr, minaddr + 12, minaddr + 14)) std::cout << rvt::green;
                if (btwn(addr, minaddr + 14, minaddr + 14 + pack.data().size()))
                    std::cout << rvt::gray;
                if (btwn(addr, minaddr + 14 + pack.data().size(),
                               minaddr + 16 + pack.data().size()))
                    std::cout << rvt::orange;
                std::cout << std::setw(2) << static_cast<int>(bytes[addr]);
                std::cout << rvt::clear << " ";
                if (addr == maxaddr - 1) ++packet_num;
            }
            else std::cout << "   ";
            if (j == 7) std::cout << " ";
        }

        std::cout << " |";

        for (size_t j = 0; j < 16; ++i, ++j)
        {
            if (i < bytes.size())
            {
                if (std::isprint(bytes[i])) std::cout << bytes[i];
                else std::cout << ".";
            }
        }

        std::cout << "|" << std::endl; 
    }
    std::cout << std::setw(8) << bytes.size() << std::endl;

    return 0;
}

