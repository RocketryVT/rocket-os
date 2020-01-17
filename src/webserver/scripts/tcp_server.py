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

s = sched.scheduler(time.time, time.sleep)
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.setblocking(False)

address = "10.0.0.46"
port = 8001
server.bind((address, port))
server.listen(10)

rospy.loginfo("Starting server at " + address + ":" + str(port))

list_of_clients = []

def broadcast(string):
	for client in list_of_clients:
		id, conn, addr = client
		try:
			conn.send(" $ " + string + "\n")
		except:
			conn.close()
			remove(client)

def ping(event):
	for client in list_of_clients:
		id, conn, addr = client
		try:
			conn.sendall("(PING " + str(datetime.now()) + ")\n")
		except Exception as e:
			conn.close()
			remove(client)

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

	conn.sendall(" $ Connection to master established.\n")
	conn.sendall(" $ You are Client #" + str(idno) + "\n")

	while True:
		try:
			message = conn.recv(2048).rstrip()
			if message:
				rospy.loginfo("Command from #" + str(idno) + ": " + message)
				pub_command.publish(message)
		except:
			pass

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
    broadcast(msg.msg)


rospy.Subscriber("/rosout", Log, get_rosout)

rospy.Timer(rospy.Duration(1), ping)
rospy.Timer(rospy.Duration(0.1), handle_connections)

rospy.spin()