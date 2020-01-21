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
from std_msgs.msg import String
from rosgraph_msgs.msg import Log

rospy.init_node("tcp_server")

pub_command = rospy.Publisher("commands", String, queue_size=10)

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.setblocking(False)

sys.argv = rospy.myargv(argv=sys.argv)

if len(sys.argv) < 3:
    rospy.logerr("Require IP address and port number as arguments.")
    exit()

address = sys.argv[1] # "192.168.1.18"
port = int(sys.argv[2]) # 8001
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
        rospy.loginfo("Sending keep-alive message.")

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

def get_rosout(msg):
    broadcast("[" + str(datetime.now()) + "] [" + str(msg.name) + "]: " + msg.msg)


rospy.Subscriber("/rosout", Log, get_rosout)

rospy.Timer(rospy.Duration(2), ping)
rospy.Timer(rospy.Duration(0.1), handle_connections)

rospy.spin()
