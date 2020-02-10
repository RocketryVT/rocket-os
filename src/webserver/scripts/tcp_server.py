#!/usr/bin/env python

# tcp_server.py

import socket
import select
import sys
from thread import *
import atexit
import signal
import time
import sched
from datetime import datetime

import rospy
from std_msgs.msg import String, Float32
from rosgraph_msgs.msg import Log

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

rospy.init_node("tcp_server", log_level=rospy.DEBUG)

pub_command = rospy.Publisher("/requested_commands", String, queue_size=10)

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.setblocking(False)

sys.argv = rospy.myargv(argv=sys.argv)

address = get_ip()
port = 8001
rospy.loginfo("Attempting to start server at " + address + ":" + str(port))

try:
    server.bind((address, port))
except:
    rospy.logerr("Error binding to IP and port provided. Please double check.")
    exit()

server.listen(10)

rospy.loginfo("Started server at " + address + ":" + str(port))

list_of_clients = []

last_broadcast = datetime.now()

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

    if seconds > 10 and len(list_of_clients):
        rospy.logdebug("Sending keep-alive message.")

def exit_handler():
    server.close()
    rospy.loginfo("Exiting.")

def signal_handler(sig, frame):
    exit()

atexit.register(exit_handler)
signal.signal(signal.SIGINT, signal_handler)

def remove(client):
    if client in list_of_clients:
        try:
            list_of_clients.remove(client)
            idno, conn, addr = client
            rospy.loginfo("Client #" + str(idno) + " has disconnected")
            rospy.loginfo(str(len(list_of_clients)) + " client(s) remaining")
        except:
            pass

def clientthread(idno, conn, addr):

    conn.sendall("[" + str(datetime.now()) + "]: Connection to master established.\n")
    conn.sendall("[" + str(datetime.now()) + "]: You are client #" + str(idno) + "\n")

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
    try:
        conn, addr = server.accept()
        idno = len(list_of_clients) + 1
        client = (idno, conn, addr)
        list_of_clients.append(client)
        rospy.loginfo("New connection: " + str((idno, addr)))
        rospy.loginfo(str(idno) + " active connection(s)")
        start_new_thread(clientthread, (idno,conn,addr))
    except:
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
    broadcast(level_to_str(msg.level) + " [" + str(datetime.now()) + "] [" + str(msg.name) + "]: " + msg.msg)

los_start_time = rospy.get_time()
los_condition = False

def publish_los(event):

    global los_start_time
    global los_condition

    num_clients = len(list_of_clients)
    los_duration = rospy.get_time() - los_start_time
    if num_clients > 0:
        los_start_time = rospy.get_time()
        los_duration = 0
        if los_condition:
            rospy.loginfo("Connection with atleast one client restored.");
        los_condition = False
    elif not los_condition:
        rospy.logwarn("LOS condition detected!");
        los_condition = True

    pub_los.publish(los_duration)

rospy.Subscriber("/rosout", Log, get_rosout)
pub_los = rospy.Publisher("/los", Float32, queue_size=10)

rospy.Timer(rospy.Duration(2), ping)
rospy.Timer(rospy.Duration(0.1), handle_connections)
rospy.Timer(rospy.Duration(1), publish_los)

rospy.spin()
