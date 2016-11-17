#!/usr/bin/env python
#
# Example python publisher using zeromq and google protocol buffers
#Messages received will be of type example_msg, defined in 
# msgs/example_msg.proto
#
#To compile the message into python, use
# roscd rustbot_translation/msgs
# protoc example_msg.proto --python_out=../src/
#
# The file created, example_msg_pb2.py should be in the same folder as the python scripts. For C# you must check how to compile for that language, and how to import the compiled message

#imports
import zmq
import time

#Use the example message defined in the folder msgs
import example_msg_pb2

#--------------------------
#Start of code
#--------------------------

#create a example_msg message and fill in the fields
#to publish
m = example_msg_pb2.Person()
m.id = 1234
m.name = "John Doe"
m.email = "jdoe@example.com"
phone = m.phone.add()
phone.number = "575-4321"
phone.type = example_msg_pb2.Person.HOME

#configure the zmq publisher
ip = "*" #* indicates clients with any ip address may listen
port = "5556" #port
topic = 777 #topic name on which to publish

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://" + ip + ":" + port)
print("Started publisher on tcp://" + ip + ":" + port + " , topic " + str(topic))

#Iterate and publish periodically
while True:

    print("Sending message:\n" + str(m) )

    #Serialization or marshalling
    msg_as_string= m.SerializeToString()
    print("Message serialized")

    #Publication
    socket.send("%d %s" % (topic, msg_as_string))
    print("Message published")

    time.sleep(1)

    
