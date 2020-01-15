import socket
import sys
import select
import atexit
import signal
import time
import sched
from datetime import datetime

s = sched.scheduler(time.time, time.sleep)
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setblocking(False)

def exit_handler():
	try:
		server.shutdown(socket.SHUT_RD)
	except:
		pass
	server.close()
	print("Exiting.")

def signal_handler(sig, frame):
	print("Caught ctrl-C")
	exit()

atexit.register(exit_handler)
signal.signal(signal.SIGINT, signal_handler)

if len(sys.argv) != 3:
	print("Correct usage: script, IP address, port number")
	exit()
address = str(sys.argv[1])
port = int(sys.argv[2])

try:
	server.connect((address, port))
except Exception as e:
	if e.errno is not 36:
		print("An error occured while attempting to connect.")
		print(str(e), type(e))
		print("Please verify that the master is available.")
		exit()

while True:

	try:
		message = server.recv(2048)
		if message and message[0] is not '(' and message[-1] is not ')':
			print(message)
	except:
		pass

	read, write, exc = select.select([sys.stdin], [], [], 0)

	if sys.stdin in read:
		try:
			server.sendall(sys.stdin.readline().splitlines()[0])
		except:
			print("Error sending command to master.")
