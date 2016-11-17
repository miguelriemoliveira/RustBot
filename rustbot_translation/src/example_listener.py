#!/usr/bin/env python
#
#   Weather update client
#   Connects SUB socket to tcp://localhost:5556
#   Collects weather updates and finds avg temp in zipcode
#

import sys
import zmq

import addressbook_pb2

#  Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)

print("Collecting updates from weather server")
socket.connect("tcp://localhost:5556")

# Subscribe to zipcode, default is NYC, 10001
zip_filter = sys.argv[1] if len(sys.argv) > 1 else "777"

# Python 2 - ascii bytes to unicode str
if isinstance(zip_filter, bytes):
    zip_filter = zip_filter.decode('ascii')
socket.setsockopt_string(zmq.SUBSCRIBE, zip_filter )

while 1:
    #string = socket.recv_string()
    #print("starting to receive")
    string = socket.recv()
    print("string is")
    print(string)
    print("finished printing string")

    person2 = addressbook_pb2.Person()
    person2.ParseFromString(string[4:])

    #print("person:\n" + str(person))
    print("person2:\n" + str(person2))

# Process 5 updates
#total_temp = 0
#for update_nbr in range(5):

    #zipcode, temperature, relhumidity = string.split()
    #total_temp += int(temperature)

#print("Average temperature for zipcode '%s' was %dF" % (
      #zip_filter, total_temp / (update_nbr+1)))
