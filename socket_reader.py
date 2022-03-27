#! /usr/bin/env python

import socket
import sys

# this program is a very simple TCP socket reader. It connects to
# a TCP socket, and continually pulls data from it until the user
# stops it or it encounters an error.
#
# It takes the IP address and port of the TCP socket as arguments.
#
# Wade Foster
# jwade109@vt.edu


def read_from_socket(sck):
    message = b""
    # continuously read until empty
    while True:
        part = b""
        try:
            part = sock.recv(1000) # read chunk
        except socket.error: # ignore all errors
            pass
        message += part # add chunk to message
        # if chunk is smaller than max size, we're done
        if len(part) < 1000:
            break
    # convert from bytes to string, ignoring non-UTF8 chars
    return message.decode("utf-8", "ignore")


if __name__ == "__main__":

    # makes sure there are enough arguments provided
    if len(sys.argv) < 3:
        print("usage:   cli_gui.py <address> <port>")
        print("example: cli_gui.py spookyscary.ddns.net 8001")
        exit()

    address = sys.argv[1]
    port = int(sys.argv[2])

    # initialize the TCP socket
    sock = socket.socket()
    try:
        sock.connect((address, port))
    except Exception as e:
        print("Failed to connect: " + str(type(e)) + ", " + str(e))
        exit()
    sock.setblocking(0) # sets to non-blocking mode

    # continuously read from the socket in 1000 byte chunks
    try:
        while True:
            message = read_from_socket(sock)
            if message:
                print(message),
    except KeyboardInterrupt:
        print("Exiting.")
        exit()
    except Exception as e:
        print("\nExiting: " + str(type(e)) + ", " + str(e))
        exit()
