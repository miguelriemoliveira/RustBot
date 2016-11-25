#!/usr/bin/env python
#
# python publisher using zeromq and google protocol buffers
#Messages received will be of type example_msg, defined in 
# msgs/SEVData.proto
#
#imports
import zmq
import time

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Use the example message defined in the folder msgs
#from google import protobuf
import SEVData_pb2
import Image_pb2

#--------------------------
# Global Vars
#--------------------------

#create a example_msg message and fill in the fields to publish
ip = "*" #* indicates clients with any ip address may listen
port = "5556" #port
topic = 777 #topic name on which to publish

context = zmq.Context()
socket = context.socket(zmq.PUB)

bridge = CvBridge()
cv_left_image = []
cv_right_image = []


#--------------------------
# Functions
#--------------------------

def leftImageReceivedCallback(data):
    #print("Received left image")
    global cv_left_image
    try:
        cv_left_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    #cv2.imshow("Left Camera", cv_image)
    #cv2.waitKey(50)

def rightImageReceivedCallback(data):
    #print("Received right image")
    global cv_right_image

    try:
        cv_right_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    #cv2.imshow("Right Camera", cv_image)
    #cv2.waitKey(50)


def cvImageToImage(cv_image):
    (height,width,channels) = cv_image.shape
    b_channel, g_channel, r_channel = cv2.split(cv_image)
    image = Image_pb2.Image()
    image.height = height
    image.width = width

    for l in range(0, height):
        for c in range(0, width):
            pixel = image.pixels.add()
            pixel.r = int(r_channel[l,c])
            pixel.g = int(g_channel[l,c])
            pixel.b = int(b_channel[l,c])

    return image

def timerCallback(event):

    print("Sending message")
    sd = SEVData_pb2.SEVData()

    # Copying left image
    (height,width,channels) = cv_left_image.shape
    b_channel, g_channel, r_channel = cv2.split(cv_left_image)

    sd.left_image.height = height
    sd.left_image.width = width

    for l in range(0, height):
        for c in range(0, width):
            pixel = sd.left_image.pixels.add()
            pixel.r = int(r_channel[l,c])
            pixel.g = int(g_channel[l,c])
            pixel.b = int(b_channel[l,c])

    # Copying right image
    (height,width,channels) = cv_right_image.shape
    b_channel, g_channel, r_channel = cv2.split(cv_right_image)

    sd.right_image.height = height
    sd.right_image.width = width

    for l in range(0, height):
        for c in range(0, width):
            pixel = sd.right_image.pixels.add()
            pixel.r = int(r_channel[l,c])
            pixel.g = int(g_channel[l,c])
            pixel.b = int(b_channel[l,c])

    #Serialization or marshalling
    msg_as_string= sd.SerializeToString()
    print("Message serialized")

    #Publication
    socket.send("%d %s" % (topic, msg_as_string))
    print("Message published")


#--------------------------
# Start of code
#--------------------------



def main(args):

    #-------------------------------------------
    # Configuration of the zmq Publisher
    #-------------------------------------------


    socket.bind("tcp://" + ip + ":" + port)
    print("Started publisher on tcp://" + ip + ":" + port + " , topic " + str(topic))

    #-------------------------------------------
    # Configuration of the ros node
    #-------------------------------------------
    rospy.init_node('sev_publisher', anonymous=True)
    left_image_sub = rospy.Subscriber("/stereo/left/image_raw", Image, leftImageReceivedCallback)
    right_image_sub = rospy.Subscriber("/stereo/right/image_raw", Image, rightImageReceivedCallback)

    #cv2.namedWindow("Left Camera")
    #cv2.namedWindow("Right Camera")

    rospy.Timer(rospy.Duration(1), timerCallback)

    #Spin infinetely
    rospy.spin()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

