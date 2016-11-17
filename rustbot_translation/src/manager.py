#!/usr/bin/env python
#
#   Weather update server
#   Binds PUB socket to tcp://*:5556
#   Publishes random weather updates
#

import zmq
from random import randrange
import addressbook_pb2
import sys 
reload(sys)  
sys.setdefaultencoding('utf8')

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
    zipcode = randrange(1, 100000)
    temperature = randrange(-80, 135)
    relhumidity = randrange(10, 60)

    #socket.send_string("%i %i %i" % (zipcode, temperature, relhumidity))
    my_string = person.SerializeToString()
    socket.send_string(my_string )


    person2 = addressbook_pb2.Person()
    person2.ParseFromString(my_string)


    print("person:\n" + str(person))
    print("person2:\n" + str(person))
