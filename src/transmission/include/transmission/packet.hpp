// packet.hpp

#ifndef RVT_PACKET_HPP
#define RVT_PACKET_HPP

#include <vector>
#include <array>
#include <set>
#include <chrono>
#include <string>
#include <iostream>
#include <utility>

namespace rvt
{

class packet
{
    public:

    static const uint8_t header_length = 16;
    static const std::string print_format;

    packet();

    packet(const packet& pack);

    packet(const std::string& fmt);

    packet(const std::array<uint8_t, header_length> &header,
           const std::vector<uint8_t> &data);

    // HEADER INFO //
    // [SYNC] [TIME] [ID] [LEN] [CS] | [DATA...]
    // 16 bytes in length

    uint16_t sync_bytes() const;
    uint16_t& sync_bytes();

    const std::chrono::system_clock::time_point& time() const;
    std::chrono::system_clock::time_point& time();

    uint16_t id() const;
    uint16_t& id();

    uint16_t data_length() const;
    uint16_t& data_length();

    uint16_t checksum() const;
    uint16_t& checksum();

    std::array<uint8_t, header_length> header() const;
    void set_header(const std::array<uint8_t, header_length> &header);

    // DATA //

    const std::vector<uint8_t>& data() const;
    std::vector<uint8_t>& data();

    // QUALITY CONTROL OPERATIONS

    uint16_t total_length() const;
 
    std::pair<bool, std::string> is_valid() const;
    
    void make_valid();

    // METADATA //

    const std::string& format() const;
    std::string& format();

    const std::string& name() const;
    std::string& name();

    private:

    // header, constant length of 16 bytes
    uint16_t _sync_bytes;
    // time is represented as 64-bit unsigned int
    std::chrono::system_clock::time_point _time;
    uint16_t _id;
    uint16_t _data_length;
    uint16_t _checksum;

    // data, variable in length
    std::vector<uint8_t> _data;

    // extra metadata, not included in binary representation
    std::string _name;
    std::string _format;
};

bool operator == (const packet& left, const packet& right);

std::vector<uint8_t>& operator << (std::vector<uint8_t> &bytes, const packet &pack);

std::vector<uint8_t>& operator >> (std::vector<uint8_t> &bytes, packet &pack);

bool nextPacket(uint16_t sync_bytes, packet &pack,
          std::vector<uint8_t>::iterator &start,
    const std::vector<uint8_t>::iterator &stop);

std::string pprintf(const std::string& fmt, const packet& pack);

std::ostream& operator << (std::ostream &os, const packet &pack);

std::vector<packet> fromFile(const std::string& binfile, uint16_t sync_bytes);

} // namespace rvt

#endif // RVT_PACKET_HPP
