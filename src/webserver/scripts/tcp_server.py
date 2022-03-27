#!/usr/bin/env python3

# tcp_server.py

import socket
import select
import sys
from _thread import *
import atexit
import signal
import time
import sched
from datetime import datetime
import bitarray

import rospy
from std_msgs.msg import String, Duration
from rosgraph_msgs.msg import Log

try:
    import Adafruit_BBIO.GPIO as gpio
except:
    gpio = None


def get_ip():

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP


def broadcast(string):

    global last_broadcast
    last_broadcast = datetime.now()

    for client in list_of_clients:
        id, conn, addr = client
        try:
            conn.send(string + "\n")
        except:
            conn.close()
            remove(client)


def ping(event):

    seconds = (datetime.now() - last_broadcast).total_seconds()

    if seconds > 2 and len(list_of_clients):
        rospy.logdebug("Sending keep-alive message.")

    if gpio:
        led = "USR3"
        gpio.output(led, gpio.HIGH)
        rospy.sleep(0.1)
        gpio.output(led, gpio.LOW)


def exit_handler():
    rospy.loginfo("Exiting.")
    time = str(datetime.now())
    broadcast(level_to_str(2) + " [" + time + "] [" + name + "]: Exiting.")
    try:
        global server
        server.close()
    except:
        pass


def signal_handler(sig, frame):
    exit()


def remove(client):
    '''
        Remove specified client from list of clients.
    '''

    if client in list_of_clients:
        try:
            list_of_clients.remove(client)
            idno, conn, addr = client
            rospy.loginfo("Client #" + str(idno) + " has disconnected")
            rospy.loginfo(str(len(list_of_clients)) + " client(s) remaining")
        except:
            pass


def clientthread(idno, conn, addr):

    while True:
        try:
            message = conn.recv(2048).rstrip()
            if message:
                rospy.loginfo("Command from #" + str(idno) + ": " + message)
                pub_command.publish(message)
        except Exception as e:
            if e.errno is 9:
                exit()


def handle_connections(event):
    '''
        Add new client connections to list of clients
    '''
    global total_connections
    try:
        conn, addr = server.accept()
        idno = total_connections
        client = (idno, conn, addr)
        list_of_clients.append(client)
        rospy.loginfo("New connection: " + str((idno, addr)))
        rospy.loginfo(str(len(list_of_clients)) + " active connection(s)")
        start_new_thread(clientthread, (idno, conn, addr))
        total_connections += 1
    except Exception as e:
        pass


def level_to_str(level):

    if level is 1:
        return '[DEBUG]'
    if level is 2:
        return '[INFO] '
    if level is 4:
        return '[WARN] '
    if level is 8:
        return '[ERROR]'
    if level is 16:
        return '[FATAL]'
    return '[?????]'


def get_rosout(msg):
    time = str(datetime.fromtimestamp(msg.header.stamp.to_sec()))
    broadcast(level_to_str(msg.level) +
              " [" + time + "] [" + str(msg.name) + "]: " + msg.msg)

    if gpio:
        led = "USR0"
        gpio.output(led, gpio.HIGH)
        rospy.sleep(0.1)
        gpio.output(led, gpio.LOW)


def publish_los(event):

    global los_start_time
    global los_condition

    num_clients = len(list_of_clients)
    los_duration = rospy.Time.now() - los_start_time

    # Client connections available
    if num_clients > 0:
        los_start_time = rospy.Time.now()
        los_duration = rospy.Duration()  # Set default duration of 0 sec & 0 nanosec
        if los_condition:
            rospy.loginfo("Connection with atleast one client restored.")
        los_condition = False

    # No client connections available
    elif not los_condition:
        rospy.logwarn("LOS condition detected!")
        los_condition = True

    pub_los.publish(los_duration)


def blink_leds(message):

    if not gpio:
        return

    word = message.data

    ba = bitarray.bitarray()
    ba.frombytes(word.encode("utf-8"))
    l = ba.tolist()

    for led in LEDs:
        gpio.output(led, gpio.LOW)

    for i in range(len(l)/4):
        states = l[i*4:(i+1)*4]
        for led, state in zip(LEDs, states):
            gpio.output(led, state)
        time.sleep(0.03)

    for led in LEDs:
        gpio.output(led, gpio.LOW)


if __name__ == "__main__":

    # register signal handler and exit handler
    atexit.register(exit_handler)
    signal.signal(signal.SIGINT, signal_handler)

    if gpio:
        LEDs = ["USR0", "USR1", "USR2", "USR3"]
        for led in LEDs:
            gpio.setup(led, gpio.OUT)
            gpio.output(led, gpio.LOW)

    rospy.init_node("tcp_server", log_level=rospy.DEBUG)
    name = rospy.get_name()

    if not gpio:
        rospy.logwarn(
            "Failed to import Adafruit_BBIO.gpio, running in desktop mode")

    list_of_clients = []
    total_connections = 0
    last_broadcast = datetime.now()
    los_start_time = rospy.Time.now()
    los_condition = False
    address = get_ip()
    port = 8001

    rospy.loginfo("Attempting to start server at " + address + ":" + str(port))

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.setblocking(False)

    try:
        server.bind((address, port))
    except:
        rospy.logerr(
            "Error binding to IP and port provided. Please double check.")
        rospy.signal_shutdown("Failed to initialize server.")
        exit()
    server.listen(10)

    rospy.loginfo("Started server at " + address + ":" + str(port))

    rospy.Subscriber("/rosout", Log, get_rosout)
    rospy.Subscriber("/requested_commands", String, blink_leds)
    pub_los = rospy.Publisher("/los", Duration, queue_size=10)
    pub_command = rospy.Publisher("/requested_commands", String, queue_size=10)

    rospy.Timer(rospy.Duration(2), ping)
    rospy.Timer(rospy.Duration(0.1), handle_connections)
    rospy.Timer(rospy.Duration(1), publish_los)

    rospy.spin()
