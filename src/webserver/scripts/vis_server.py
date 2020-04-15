#!/usr/bin/env python

# vis_server.py

import socket
import select
import sys
from thread import *
import atexit
import signal
import time
import sched
from datetime import datetime
import bitarray

import rospy
from std_msgs.msg import String, Float32
from rosgraph_msgs.msg import Log
import Adafruit_BBIO.GPIO as gpio

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        #Changed to .255
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except:
        #And .1
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP


def broadcast(string):
    rospy.logdebug(list_of_clients)
    rospy.loginfo(string)

    global last_broadcast
    last_broadcast = datetime.now()

    for client in list_of_clients:
        id, conn, addr = client
        try:
            conn.send(string.data)
        except Exception as e:
            print(e)
            conn.close()
            remove(client)



def exit_handler():
    rospy.loginfo("Exiting.")
    time = str(datetime.now())
    broadcast("[" + time + "] [" + name + "] Exiting.")
    try:
        global server
        server.close()
    except:
        pass


def signal_handler(sig, frame):
    exit()


def remove(client):
    global total_connections
    if client in list_of_clients:
        try:
            list_of_clients.remove(client)
            idno, conn, addr = client
            rospy.loginfo("Client #" + str(idno) + " has disconnected")
            rospy.loginfo(str(len(list_of_clients)) + " client(s) remaining")
            rospy.loginfo("Got to end of remove")
        except:
            pass


def clientthread(idno, conn, addr):
    while True:
        try:
            message = conn.recv(2048).rstrip()
            if message:
                rospy.loginfo("Command from #" + str(idno) + ": " + message)
                pub_msg.publish(message)
                rospy.loginfo("Got to end")
        except Exception as e:
            if e.errno is 9:
                exit()


def handle_connections(event):
    global total_connections
    try:
        conn, addr = server.accept()
        rospy.loginfo("Got to accept.")
        idno = total_connections
        client = (idno, conn, addr)
        list_of_clients.append(client)
        rospy.loginfo("New connection: " + str((idno, addr)))
        rospy.loginfo(str(len(list_of_clients)) + " active connection(s)")
        start_new_thread(clientthread, (idno,conn,addr))
        total_connections += 1
        rospy.loginfo("got to end of handle")
    except Exception as e:
        pass

def ping(event):

    seconds = (datetime.now() - last_broadcast).total_seconds()

    if seconds > 2 and len(list_of_clients):
        rospy.logdebug("Sending keep-alive message.")
        broadcast("Sending keep-alive message.")
    

if __name__ == "__main__":
   
    # register signal handler and exit handler 
    atexit.register(exit_handler)
    signal.signal(signal.SIGINT, signal_handler)


    #Init server node
    rospy.init_node("vis_server", log_level=rospy.DEBUG)
    name = rospy.get_name()

    list_of_clients = []
    total_connections = 0
    last_broadcast = datetime.now()
    address = get_ip()
    
    #Port here is 8002.
    port = 8002

    rospy.loginfo("Attempting to start server at " + address + ":" + str(port))

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.setblocking(False)

    try:
        server.bind((address, port))
    except:
        rospy.logerr("Error binding to IP and port provided. Please double check.")
        rospy.signal_shutdown("Failed to initialize server.")
        exit()
    server.listen(10)

    rospy.loginfo("Started server at " + address + ":" + str(port))

    #Subbed to vis_update instead.
    rospy.Subscriber("/vis_update", String, broadcast)
    rospy.loginfo("Subscribed")
    pub_msg = rospy.Publisher("/vis_update", String, queue_size=10)

    rospy.Timer(rospy.Duration(0.1), handle_connections)
    rospy.Timer(rospy.Duration(2), ping)


    rospy.spin()
