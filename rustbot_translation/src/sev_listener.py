#!/usr/bin/env python
#
#imports
import zmq
import time
import numpy as np

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import copy

import SEVData_pb2
import Image_pb2

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

print("Started subscriber on tcp://" + ip + ":" + port + " , topic " + str(topic))

sd = SEVData_pb2.SEVData()

def ImageToCVImage(image):
    height = image.height
    width = image.width

    #print("height = " + str(height) + " width=" + str(width))
    #b_channel = np.zeros((height,width,1), np.uint8)
    #r_channel = np.zeros((height,width,1), np.uint8)
    #g_channel = np.zeros((height,width,1), np.uint8)

    #for l in range(0, height):
        #for c in range(0, width):
            #idx = c + l * width
            #pixel = image.pixels[idx]

            #r_channel[l,c] = pixel.r
            #g_channel[l,c] = pixel.g
            #b_channel[l,c] = pixel.b


    cv_image = np.zeros((height,width,3), np.uint8)

    #cv_image.data = copy.deepcopy(image.data)
    cv_image.data = image.data
    #cv_image = cv2.merge((b_channel, g_channel, r_channel)) 
    return cv_image


def main(args):
    global socket
    socket.connect("tcp://" + ip + ":" + port)
    socket.setsockopt_string(zmq.SUBSCRIBE, topic )

    cv2.namedWindow("Listener Left Camera")
    cv2.namedWindow("Listener Right Camera")
    count = 0
    height = width = 0
    cv_left_image = [] 



    while True:

        #maxpkglen = 1024 * 1024 # original maxpkglen = 1024
        #s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #s.sendto(req, (server, port))
        #rsp, server = s.recvfrom(maxpkglen)
        #s.close()
        #Receive message
        #n = 1024
        #buf = ''
        #while n > 0:
            #data = socket.recv(n)
            #if data == '':
                #raise RuntimeError('unexpected connection close')
            #buf += data
            #n -= len(data)
            
        #message = buf
        message = socket.recv(10*1024)
        

        print("Message received with " + str(len(message)))

        #Deserialization or unmarshalling
        #We use message[4:] because we know the first four bytes are
        #"777 " and we only want to give the data to the parser (not the topic name
        sd.ParseFromString(message[4:])
        #sd.ParseFromString(message.data)
        #print("Received message:\n" + str(m) )

        if height == 0:
            height = sd.left_image.height
            width = sd.left_image.width
            cv_left_image = np.zeros((height,width,3), np.uint8)
            print("Initialized images with " + str(height) + "x" + str(width))


        #cv_left_image = ImageToCVImage(sd.left_image)
        cv_left_image.data = sd.left_image.data
        #cv_right_image = ImageToCVImage(sd.right_image)
        cv2.imshow("Listener Left Camera", cv_left_image)
        #cv2.imshow("Listener Right Camera", cv_right_image)
        cv2.waitKey(30)

        print("Count = " + str(count))
        count = count + 1
        print("Got cv_image")
        #del cv_left_image
        #time.sleep(5.0)

if __name__ == '__main__':
    main(sys.argv)

