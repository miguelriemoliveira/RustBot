#!/usr/bin/env python
#
# python publisher using zeromq and google protocol buffers
#Messages received will be of type example_msg, defined in 
# msgs/SEVData.proto
#
#imports
import zmq
import time

import numpy as np

import roslib
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image
from std_msgs.msg import String

import google.protobuf
import SEVData_pb2
import PointField_pb2

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
point_cloud2_msg = []


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

def pointcloudReceivedCallback(data):
    #print("Received point cloud")
    global point_cloud2_msg 
    point_cloud2_msg = data
    #point_cloud2_msg = copy.deepcopy(data)

def timerCallback(event):

    #Start preparing message to send
    start_time = time.time()
    sd = SEVData_pb2.SEVData()

    # Copying left image
    (sd.left_image.height,sd.left_image.width ,_) = cv_left_image.shape
    sd.left_image.data = bytes(cv_left_image.data)

    # Copying right image
    (sd.right_image.height,sd.right_image.width ,_) = cv_right_image.shape
    sd.right_image.data = bytes(cv_right_image.data)

    # Copying the point cloud
    sd.point_cloud.height = point_cloud2_msg.height
    sd.point_cloud.width = point_cloud2_msg.width

    for field in point_cloud2_msg.fields:
        point_field = sd.point_cloud.fields.add()
        point_field.name = field.name
        point_field.offset = field.offset
        point_field.datatype = field.datatype
        point_field.count = field.count

    sd.point_cloud.is_bigendian = point_cloud2_msg.is_bigendian
    sd.point_cloud.point_step = point_cloud2_msg.point_step
    sd.point_cloud.row_step = point_cloud2_msg.row_step
    sd.point_cloud.data = point_cloud2_msg.data


    elapsed_time = time.time() - start_time
    print("Finished copying messages to SEVData message in " + str(elapsed_time))

    #Serialization or marshalling
    msg_as_string= sd.SerializeToString()
    print("Message serialized")

    #Publication
    socket.send("%d %s" % (topic, msg_as_string))
    print("Sending message")


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

    rospy.Subscriber("/stereo/points2", PointCloud2, pointcloudReceivedCallback)

    #cv2.namedWindow("Left Camera")
    #cv2.namedWindow("Right Camera")

    #TODO This wait is to try to get at least one message of each time before publishing the message. Its a blind wait so sometimes it does not work. In the future, the condition above should be checked before sending
    rospy.Timer(rospy.Duration(1.5), timerCallback)

    #Spin infinetely
    rospy.spin()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

