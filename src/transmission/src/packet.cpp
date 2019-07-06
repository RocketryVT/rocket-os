// packet.cpp

#include <transmission/serial.hpp>
#include <transmission/packet.hpp>
#include <transmission/util.hpp>

#include <vector>
#include <set>
#include <chrono>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <iterator>
#include <sstream>

namespace rvt
{

const std::string packet::print_format = "[%@][%#][%$]: %s";

packet::packet() :
    _sync_bytes(0),
    _time{}, _id(0), _data_length(0),
    _checksum(0), _data(),
    _name(), _format() { }

packet::packet(const packet& pack) :
    _sync_bytes(pack.sync_bytes()),
    _time(pack.time()),
    _id(pack.id()),
    _data_length(pack.data_length()),
    _checksum(pack.checksum()),
    _data(pack.data()),
    _name(pack.name()),
    _format(pack.format()) { }

packet::packet(const std::array<uint8_t, header_length> &header,
               const std::vector<uint8_t> &data) : packet() 
{
    _data = data;
    set_header(header);
    _data_length = data.size();
    make_valid();
}

packet::packet(const std::string& fmt) : packet()
{
    _time = std::chrono::system_clock::now();

    auto tokens = split_quoted(fmt);

    for (auto token : tokens)
    {
        std::string value = "";
        if (begins_with(token, "!") && token.length() > 1)
        {
            value = token.substr(1);
            _sync_bytes = std::stoull(value, 0, 16);
        }
        else if (begins_with(token, "@") && token.length() > 1)
        {
            value = token.substr(1);
            double seconds = std::stod(value);
            auto dur = std::chrono::milliseconds((uint64_t) (seconds*1000));
            _time = std::chrono::system_clock::time_point{} + dur;
        }
        else if (begins_with(token, "#") && token.length() > 1)
        {
            value = token.substr(1);
            _id = std::stoull(value);
        }
        else if (begins_with(token, "$") && token.length() > 1)
        {
            _name = token.substr(1);
        }
        else if (begins_with(token, "0x") && token.length() > 2)
        {
            value = token.substr(2);
            uint8_t byte_value = std::stoull(value, 0, 16);
            std::vector<uint8_t> bytes{byte_value};
            _data.insert(_data.end(), bytes.begin(), bytes.end());
            _format += "0x ";
        }
        else if (token.length() >= 2 &&
               ((begins_with(token, "'")  && ends_with(token, "'")) ||
                (begins_with(token, "\"") && ends_with(token, "\""))))
        {
            value = token.substr(1, token.length() - 2);
            std::vector<uint8_t> bytes(value.begin(), value.end());
            bytes.push_back(0);
            _data.insert(_data.end(), bytes.begin(), bytes.end());
            _format += "s ";
        }
        else if (ends_with(token, "u8") && token.length() > 2)
        {
            value = token.substr(0, token.length() - 2);
            uint8_t byte_value = std::stoull(value);
            std::vector<uint8_t> bytes{byte_value};
            _data.insert(_data.end(), bytes.begin(), bytes.end());
            _format += "u8 ";
        }
        else if (ends_with(token, "u16"))
        {
            value = token.substr(0, token.length() - 3);
            uint16_t num = std::stoull(value);
            std::vector<uint8_t> bytes;
            bytes << num;
            _data.insert(_data.end(), bytes.begin(), bytes.end());
            _format += "u16 ";
        }
        else if (ends_with(token, "u32"))
        {
            value = token.substr(0, token.length() - 3);
            uint32_t num = std::stoull(value);
            std::vector<uint8_t> bytes;
            bytes << num;
            _data.insert(_data.end(), bytes.begin(), bytes.end());
            _format += "u32 ";
        }
        else if (ends_with(token, "u64"))
        {
            value = token.substr(0, token.length() - 3);
            uint64_t num = std::stoull(value);
            std::vector<uint8_t> bytes;
            bytes << num;
            _data.insert(_data.end(), bytes.begin(), bytes.end());
            _format += "u64 ";
        }
        else if (ends_with(token, "n8"))
        {
            value = token.substr(0, token.length() - 2);
            int8_t num = std::stoll(value);
            std::vector<uint8_t> bytes;
            bytes << num;
            _data.insert(_data.end(), bytes.begin(), bytes.end());
            _format += "n8 ";
        }
        else if (ends_with(token, "n16"))
        {
            value = token.substr(0, token.length() - 3);
            int16_t num = std::stoll(value);
            std::vector<uint8_t> bytes;
            bytes << num;
            _data.insert(_data.end(), bytes.begin(), bytes.end());
            _format += "n16 ";
        }
        else if (ends_with(token, "n32"))
        {
            value = token.substr(0, token.length() - 3);
            int32_t num = std::stoll(value);
            std::vector<uint8_t> bytes;
            bytes << num;
            _data.insert(_data.end(), bytes.begin(), bytes.end());
            _format += "n32 ";
        } 
        else if (ends_with(token, "n64"))
        {
            value = token.substr(0, token.length() - 3);
            int64_t num = std::stoll(value);
            std::vector<uint8_t> bytes;
            bytes << num;
            _data.insert(_data.end(), bytes.begin(), bytes.end());
            _format += "n64 ";
        }
        else if (ends_with(token, "f"))
        {
            value = token.substr(0, token.length() - 1);
            float num = std::stof(value);
            std::vector<uint8_t> bytes;
            bytes << num;
            _data.insert(_data.end(), bytes.begin(), bytes.end());
            _format += "f ";
        }
        else if (ends_with(token, "d"))
        {
            value = token.substr(0, token.length() - 1);
            double num = std::stod(value);
            std::vector<uint8_t> bytes;
            bytes << num;
            _data.insert(_data.end(), bytes.begin(), bytes.end());
            _format += "d ";
        }
        else
        {
            std::cout << "Unrecognized token: '"
                      << token << "'" << std::endl;
        }
    }
    _data_length = _data.size();
    make_valid();
}

uint16_t packet::sync_bytes() const
{
    return _sync_bytes;
}

uint16_t& packet::sync_bytes()
{
    return _sync_bytes;
}

const std::chrono::system_clock::time_point& packet::time() const
{
    return _time;
}

std::chrono::system_clock::time_point& packet::time()
{
    return _time;
}

uint16_t packet::id() const
{
    return _id;
}

uint16_t& packet::id()
{
    return _id;
}

const std::string& packet::name() const
{
    return _name;
}

std::string& packet::name()
{
    return _name;
}

uint16_t packet::checksum() const
{
    return _checksum;
}

uint16_t& packet::checksum()
{
    return _checksum;
}

uint16_t packet::data_length() const
{
    return _data_length;
}

uint16_t& packet::data_length()
{
    return _data_length;
}

uint16_t packet::total_length() const
{
    return packet::header_length + _data.size();
}

void packet::set_header(const std::array<uint8_t, header_length> &header)
{
    auto copy(header);
    copy >> _sync_bytes >> _time >> _id >> _data_length >> _checksum;
}

std::array<uint8_t, 16> packet::header() const
{
    std::array<uint8_t, 0> arr;
    return arr << _sync_bytes << _time << _id
        << _data_length << _checksum;
}

const std::vector<uint8_t>& packet::data() const
{
    return _data;
}

std::vector<uint8_t>& packet::data()
{
    return _data;
}

const std::string& packet::format() const
{
    return _format;
}

std::string& packet::format()
{
    return _format;
}

std::pair<bool, std::string> packet::is_valid() const
{
    std::vector<uint8_t> hexdump;
    hexdump << *this;
    if (_data_length != _data.size())
    {
        return { false, "Header data_length disagrees"
                        " with member function size()" };
    }
    if (0 != xorchecksum(hexdump))
    {
        return { false, "Checksum mismatch" };
    }
    return { true, "" };
}

void packet::make_valid()
{
    _data_length = _data.size();
    _checksum = 0;
    std::vector<uint8_t> hexdump;
    hexdump << *this;
    _checksum = xorchecksum(hexdump);
}

// NON MEMBER FUNCTIONS ---------------------------------------------

bool nextPacket(uint16_t sync_bytes, packet &pack,
          std::vector<uint8_t>::iterator &start,
    const std::vector<uint8_t>::iterator &stop)
{
    for (; start < stop; ++start)
    {
        auto read_iter(start);
        // not enough data for a min-size packet
        if (stop - read_iter < packet::header_length)
        {
            std::cout << "Not enough data (1)" << std::endl;
            return false;
        }

        std::array<uint8_t, packet::header_length> buffer;
        std::copy(read_iter, read_iter + packet::header_length, begin(buffer));
        pack.set_header(buffer);

        if (sync_bytes != pack.sync_bytes())
        {
            std::cout << "Bad sync bytes" << std::endl;
            continue;
        }
       
        // increment read iter to end of packet header
        read_iter += packet::header_length;

        if (stop - read_iter < pack.data_length())
        {
            std::cout << "Not enough data (2)" << std::endl;
            // not enough data in the buffer 
            return false;
        }

        pack.data().clear();
        pack.data().insert(pack.data().end(),
            read_iter, read_iter + pack.data_length());
     
        std::cout << pack << std::endl;
 
        if (pack.is_valid().first)
        {
            start += packet::header_length + pack.data_length();
            return true;
        }
        else
        {
            std::cerr << "Problem with this packet: "
                << pack << std::endl;
        }
    }
    return false;
}

std::vector<uint8_t>& operator << (std::vector<uint8_t> &vec,
                                   const packet &pack)
{
    return vec << pack.header() << pack.data();
}

std::vector<uint8_t>& operator >> (std::vector<uint8_t> &vec,
                                   packet &pack)
{
    std::array<uint8_t, packet::header_length> header;
    vec >> header;
    pack.set_header(header);

    if (pack.data_length() > vec.size()) pack.data_length() = 0;

    pack.data().insert(pack.data().begin(), vec.begin(), vec.begin() + pack.data_length());
    vec.erase(vec.begin(), vec.begin() + pack.data_length());

    return vec;
}

std::ostream& operator << (std::ostream &os, const packet &pack)
{
    using namespace std::chrono;

    auto since_epoch = pack.time().time_since_epoch();
    auto sec = duration_cast<seconds>(since_epoch);
    auto ms = duration_cast<milliseconds>(since_epoch) - sec;

    std::stringstream ss;
    ss << std::setfill(' ') << std::setw(18) << std::dec << std::right
       << "time: " << std::right << std::setfill('0')
       << std::setw(10) << sec.count() << "."
       << std::setw(3) << std::right << ms.count() << " -- ";

    ss << dateString(pack.time()) << std::endl << std::setfill(' ')
       << std::setw(18) << "sync bytes: " << std::hex << std::setw(4)
       << std::setfill('0') << pack.sync_bytes() << std::endl;

    std::vector<uint8_t> bytestring;
    bytestring << pack;

    ss << std::setfill(' ') << std::setw(18) << "id: " << std::dec << pack.id()
       << std::endl << std::setw(18) << "length: " << pack.data_length()
       << " / " << pack.total_length() << std::endl
       << std::setw(18) << "checksum: " << std::hex << std::setw(4)
       << std::setfill('0') << pack.checksum() << std::endl << std::setfill(' ')
       << std::setw(18) << "format: " << pack.format() << std::endl
       << std::setw(18) << "name: " << pack.name() << std::endl
       << std::setw(18) << "string: " << pprintf("%s", pack) << std::endl
       << std::setw(18) << "valid: " << std::boolalpha;

    auto is_valid = pack.is_valid();
    ss << is_valid.first;
    if (!is_valid.first)
        ss << " -- " << is_valid.second;
    ss << std::endl;

    auto print_vec = [&] (const std::string& name,
                          const std::vector<uint8_t> data)
    {
        ss << std::setfill(' ') << std::setw(18) << std::right << name; 
        for (size_t i = 0; i < data.size(); ++i)
        {
            uint8_t e = data[i];
            ss << std::right << std::hex << std::setw(2)
                << std::setfill('0') << static_cast<int>(e) << " ";
            if (i % 16 == 7) ss << " ";
            if (i % 16 == 15 && i < data.size() - 1)
                ss << std::endl << std::setw(18) << std::setfill(' ') << "";
        }
        ss << std::endl;
    };

    auto header = pack.header();
    std::vector<uint8_t> vec(begin(header), end(header));
    print_vec("header: ", vec);
    print_vec("data: ", pack.data());

    return os << ss.str();
}

std::vector<packet> fromFile(const std::string &ifname, uint16_t sync_bytes)
{
    std::ifstream infile(ifname, std::ios::binary);
    std::vector<uint8_t> bytes(
        std::istreambuf_iterator<char>{infile}, {});

    auto read_ptr = begin(bytes);

    std::vector<packet> packets;
    packet pack;
    while (nextPacket(sync_bytes, pack, read_ptr, end(bytes)))
    {
        packets.push_back(pack);
    }
    return packets;
}

} // namespace rvt

