#import SocketServer
import socket
import sys
import os, os.path
import time


s = socket.socket()
if os.path.exists("echo_socket"):
	print "Connecting to C...\n"
	client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
	while(client.connect_ex("echo_socket")!=0):
		time.sleep(0.1)
	print "Connected to C\n"


while(1):
	time.sleep(0.25)
	try:
		s.send("R")	
		SDRdata = s.recv(1024)
	except:
		print "Disconnected from SDR\n"
		while(s.connect_ex(('192.168.10.2',7000))!=0):
			time.sleep(0.1)
		s.send("R")	
		SDRdata = s.recv(1024)
		print "Reconnected to SDR\n"
	print "Data sent to C: {0}\n".format(SDRdata)
	#client.send("{0}".format(SDRdata))
	try:
		client_recv = client.recv(1024)
		print client_recv
		if(client_recv == "1"):	
			client.send("{0}".format(SDRdata))
	except:
		print "Disconnected from C program\n"
		while(client.connect_ex("echo_socket")!=0):
			time.sleep(0.1)
		client_recv = client.recv(1024)
		if(client_recv == "1"):	
			client.send("{0}".format(SDRdata))
		print "Reconnected to C\n"
	
