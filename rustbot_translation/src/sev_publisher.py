#!/usr/bin/env python
#
# python publisher using zeromq and google protocol buffers
#Messages received will be of type example_msg, defined in 
# msgs/SEVData.proto
#
#imports
import zmq
import time

#Use the example message defined in the folder msgs
import sevdata_

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

    
