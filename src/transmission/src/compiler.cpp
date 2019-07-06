// corrections.cpp

#include <transmission/packet.hpp>
#include <transmission/util.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <chrono>

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << std::string(argv[0]) << ": usage: "
            << std::string(argv[0]) << " [text file] [bin file]"
            << std::endl;
        return 1;
    }

    std::ifstream infile(argv[1]);
    std::ofstream outfile(argv[2], std::ios::binary);

    if (!infile)
    {
        std::cerr << std::string(argv[0]) << ": failed to open '"
            << std::string(argv[1]) << "'" << std::endl;
        return 1;
    }
    
    if (!outfile)
    {
        std::cerr << std::string(argv[0]) << ": failed to open '"
            << std::string(argv[2]) << "'" << std::endl;
        return 1;
    }

    std::vector<uint8_t> output_buffer;

    std::string line;
    size_t linenumber = 1;
    for (; std::getline(infile, line); linenumber++)
    {
        if (line.length() == 0) continue;
        if (rvt::begins_with(line, "//")) continue; // comments

        try
        {
            output_buffer << rvt::packet(line);
        }
        catch (const std::invalid_argument &e)
        {
            std::cerr << std::string(argv[1]) << ": error parsing line "
                << linenumber << " (invalid argument: "
                << e.what() << "):\n\n   "
                << line << "\n" << std::endl;
            return false;
        }
    }

    outfile.write(reinterpret_cast<char*>(output_buffer.data()),
                  output_buffer.size());

    return 0;
}
