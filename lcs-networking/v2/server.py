import socket
import select
import sys
from thread import *
import atexit
import signal
import time
import sched
from datetime import datetime

s = sched.scheduler(time.time, time.sleep)
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.setblocking(False)

address = "10.0.0.55"
port = 8001
server.bind((address, port))
server.listen(10)

print("Starting server at " + address + ":" + str(port))

list_of_clients = []

def broadcast(string):
	for id, conn, addr in list_of_clients:
		try:
			conn.send(" $ " + string)
		except:
			conn.close()
			remove(conn)

def ping():
	for id, conn, addr in list_of_clients:
		try:
			conn.sendall("(PING " + str(datetime.now()) + ")")
		except Exception as e:
			conn.close()
			remove(conn)
	s.enter(1, 1, ping, ())

def exit_handler():
	server.close()
	print("Exiting.")

def signal_handler(sig, frame):
	exit()

atexit.register(exit_handler)
signal.signal(signal.SIGINT, signal_handler)

def remove(connection):
	for client in list_of_clients:
		idno, conn, addr = client
		if conn == connection:
			print("Client #" + str(idno) + " has disconnected")
			list_of_clients.remove(client)
			print(str(len(list_of_clients)) + " client(s) remaining")
			break


def clientthread(idno, conn, addr):

	conn.send(" $ Connection to master established.")
	conn.send(" $ You are Client #" + str(idno))

	while True:
		try:
			message = conn.recv(2048)
			if message:
				print("Command from #" + str(idno) + ": " + message)
				broadcast("Command from #" + str(idno) + ": " + message)
		except:
			pass

def handle_connections():
	try:
		conn, addr = server.accept()
		idno = len(list_of_clients) + 1
		client = (idno, conn, addr)
		list_of_clients.append([idno, conn, addr])
		print("New connection: " + str((idno, addr)))
		print(str(idno) + " active connection(s)")
		start_new_thread(clientthread, (idno,conn,addr))
	except:
		pass
	s.enter(0.1, 1, handle_connections, ())


s.enter(0, 1, ping, ())
s.enter(0, 1, handle_connections, ())
s.run()
