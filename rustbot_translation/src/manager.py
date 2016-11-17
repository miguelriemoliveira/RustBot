#!/usr/bin/env python
#
#Example SEV project

import zmq
from random import randrange
import addressbook_pb2

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5556")


person = addressbook_pb2.Person()
person.id = 1234
person.name = "John Doe"
person.email = "jdoe@example.com"
phone = person.phone.add()
phone.number = "555-4321"
phone.type = addressbook_pb2.Person.HOME


while True:

    zipcode = randrange(1, 5)
    #zipcode = 10001
    temperature = randrange(-80, 135)
    relhumidity = randrange(10, 60)

    #socket.send_string("%i %i %i" % (zipcode, temperature, relhumidity))
    my_string = person.SerializeToString()
    #socket.send_string(my_string )
    #socket.send_string("%s" % my_string )
    #socket.send_data( my_string )
    print("publishing message")
    #socket.send( my_string, copy=True )

    topic = 777
    socket.send("%d %s" % (topic, my_string))
    print("finished publishing")

    
