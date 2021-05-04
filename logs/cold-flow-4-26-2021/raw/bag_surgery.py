#! /usr/bin/env python3

import argparse
import struct
from collections import deque

from sensors.msg import SensorReading

print(dir(SensorReading))

parser = argparse.ArgumentParser(description='Perform bag surgery.')
parser.add_argument("input")
args = parser.parse_args()

binary = deque(open(args.input, "rb").read())
num_bytes = len(binary)

BAG_HEADER = "#ROSBAG V2.0\n"

def get_n(byte_queue, n):
    bytes = []
    while byte_queue and len(bytes) < n:
        bytes.append(byte_queue.popleft())
    if len(bytes) < n:
        print(f"Warning: expected {n} bytes, got {len(bytes)}.")
    return bytes

def unsigned_int(byte_queue):
    bytes = get_n(byte_queue, 4)
    return int.from_bytes(bytes, byteorder='little')

def unsigned_long(byte_queue):
    bytes = get_n(byte_queue, 8)
    return int.from_bytes(bytes, byteorder='little')

def extract_bag_header(byte_queue):
    bytes = get_n(byte_queue, len(BAG_HEADER))
    return "".join([chr(x) for x in bytes])

def opcode2str(opcode):
    opcodes = [
        "", "",
        "MESSAGE DATA",
        "BAG HEADER",
        "INDEX DATA",
        "CHUNK",
        "CHUNK INFO",
        "CONNECTION"]
    if opcode < 2 or opcode >= len(opcodes):
        print(f"Unhandled opcode: {opcode}")
        return "?"
    return opcodes[opcode]

def parse_header_member(name, data):
    if name == "op":
        return opcode2str(int(data[0]))
    elif name in ["start_time", "time", "end_time"]:
        deq = deque(data)
        secs = unsigned_int(deq)
        nsecs = unsigned_int(deq)
        return (secs, nsecs)
    elif name in ["message_definition", "latching", "type", "callerid", "topic", "compression"]:
        return "".join([chr(x) for x in data])
    elif name in ["conn", "ver", "size", "count", "chunk_count", "conn_count"]:
        return unsigned_int(data)
    elif name in ["chunk_pos", "index_pos"]:
        return unsigned_long(data)
    elif name in ["md5sum"]:
        return "".join(chr(x) for x in data)
    else:
        print(f">> Unhandled member: {name}")
        return data

def extract_header_data(byte_queue, length):
    data = deque(get_n(byte_queue, length))
    ret = {}
    while data:
        l = unsigned_int(data)
        member_data = deque(get_n(data, l))
        name = ""
        while chr(member_data[0]) != "=":
            name += chr(member_data.popleft())
        member_data.popleft()
        member_value = parse_header_member(name, member_data)
        ret[name] = member_value
    return ret

def extract_record(byte_queue):
    ret = {}
    header_len = unsigned_int(byte_queue)
    if not header_len:
        return None
    header = extract_header_data(byte_queue, header_len)
    ret["header"] = header
    data_len = unsigned_int(byte_queue)
    data = get_n(byte_queue, data_len)
    ret["data"] = bytes(data)
    if "op" in header and header["op"] == "CONNECTION":
        ret["data"] = extract_header_data(deque(data), data_len)
    return ret

header = extract_bag_header(binary)
if header != BAG_HEADER:
    print("Not a bag file!")
    exit()

sensor_conn_ids = []
topics = {}

records = 0
while binary:
    record = extract_record(binary)
    if record:
        # print(f"{records}: {record}")
        if record["header"]["op"] == "CONNECTION":
            if record["data"]["md5sum"] == SensorReading._md5sum:
                sensor_conn_ids.append(record["header"]["conn"])
                topics[record["header"]["conn"]] = record["header"]["topic"]
        if record["header"]["op"] == "MESSAGE DATA" and record["header"]["conn"] in sensor_conn_ids:
            if topics[record["header"]["conn"]] != "/sensors/ox_tank_transducer":
                continue
            sr = SensorReading()
            sr.deserialize(record["data"])
            # print(sr)

            fsecs = sr.header.stamp.to_sec()
            print(f"{fsecs} {sr.reading}")

            # input()
        records += 1
print(f"Parsed {records} records.")

