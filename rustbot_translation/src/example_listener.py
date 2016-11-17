#!/usr/bin/env python
#
# Example python listener using zeromq and google protocol buffers
#Messages received will be of type example_msg, defined in 
# msgs/example_msg.proto
#
#To compile the message into python, use
# roscd rustbot_translation/msgs
# protoc example_msg.proto --python_out=../src/
#
# The file created, example_msg_pb2.py should be in the same folder as the python scripts. For C# you must check how to compile for that language, and how to import the compiled message

import sys
import zmq

#Use the example message defined in the folder msgs
import example_msg_pb2

#--------------------------
#Start of code
#--------------------------

#configure the zmq publisher
ip = "localhost" #* indicates clients with any ip address may listen
port = "5556" #port
topic = "777" #topic name on which to publish
if isinstance(topic, bytes): # Python 2 - ascii bytes to unicode str
    topic = topic.decode('ascii')

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://" + ip + ":" + port)
socket.setsockopt_string(zmq.SUBSCRIBE, topic )

print("Started subscriber on tcp://" + ip + ":" + port + " , topic " + str(topic))

#Create a new instance of Person to unmarshall the data to
m = example_msg_pb2.Person()

while True:

    #Receive message
    message = socket.recv()
    print("Message received")

    #Deserialization or unmarshalling
    #We use message[4:] because we know the first four bytes are
    #"777 " and we only want to give the data to the parser (not the topic name
    m.ParseFromString(message[4:])
    print("Received message:\n" + str(m) )

