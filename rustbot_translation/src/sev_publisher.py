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

from sensor_msgs.msg import Image, PointCloud2, PointField, NavSatFix
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import google.protobuf
import SEVData_pb2

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
left_image_header = []
cv_right_image = []
right_image_header = []
point_cloud2_msg = []
nav_sat_fix_msg = []
odometry_msg = []

rcv_left_image = False
rcv_right_image = False
rcv_point_cloud = False
rcv_nav_sat_fix = False
rcv_odometry = False

#--------------------------
# Functions
#--------------------------

def leftImageReceivedCallback(data):
    #print("Received left image")
    global cv_left_image
    global left_image_header
    global rcv_left_image
    try:
        cv_left_image = bridge.imgmsg_to_cv2(data, "bgr8")
        left_image_header = data.header
        rcv_left_image = True
    except CvBridgeError as e:
        print(e)

    #cv2.imshow("Left Camera", cv_image)
    #cv2.waitKey(50)

def rightImageReceivedCallback(data):
    #print("Received right image")
    global cv_right_image
    global right_image_header
    global rcv_right_image
    rcv_right_image = True
    try:
        cv_right_image = bridge.imgmsg_to_cv2(data, "bgr8")
        right_image_header = data.header
    except CvBridgeError as e:
        print(e)

    #cv2.imshow("Right Camera", cv_image)
    #cv2.waitKey(50)

def pointcloudReceivedCallback(data):
    global point_cloud2_msg 
    point_cloud2_msg = data
    global rcv_point_cloud
    rcv_point_cloud = True

def navSatFixReceivedCallback(data):
    global nav_sat_fix_msg
    nav_sat_fix_msg = data
    global rcv_nav_sat_fix
    rcv_nav_sat_fix = True

def odometryReceivedCallback(data):
    global odometry_msg
    odometry_msg = data
    global rcv_odometry
    rcv_odometry = True

def headerMsg2Proto(msg, proto):
    proto.frame_id = msg.frame_id
    proto.stamp.seconds = msg.stamp.secs
    proto.stamp.nanos = msg.stamp.nsecs

def cvImage2Proto(image, header, proto):
    headerMsg2Proto(header, proto.header)
    (proto.height, proto.width, _) = image.shape
    proto.data = bytes(image.data)

def pointCloudMsg2Proto(msg, proto):
    headerMsg2Proto(msg.header, proto.header)
    proto.height = msg.height
    proto.width = msg.width

    for field in msg.fields:
        point_field = proto.fields.add()
        point_field.name = field.name
        point_field.offset = field.offset
        point_field.datatype = field.datatype
        point_field.count = field.count

    proto.is_bigendian = msg.is_bigendian
    proto.point_step = msg.point_step
    proto.row_step = msg.row_step
    proto.data = msg.data

def navSatFixMsg2Proto(msg, proto):
    headerMsg2Proto(msg.header, proto.header)
    proto.latitude = msg.latitude
    proto.longitude = msg.longitude
    proto.altitude = msg.altitude
    proto.status.status = msg.status.status
    proto.status.service = msg.status.service

def odometryMsg2Proto(msg, proto):
    headerMsg2Proto(msg.header, proto.header)
    proto.child_frame_id = msg.child_frame_id
    proto.pose.pose.position.x = msg.pose.pose.position.x
    proto.pose.pose.position.y = msg.pose.pose.position.y
    proto.pose.pose.position.z = msg.pose.pose.position.z

    proto.pose.pose.orientation.x = msg.pose.pose.orientation.x
    proto.pose.pose.orientation.y = msg.pose.pose.orientation.y
    proto.pose.pose.orientation.z = msg.pose.pose.orientation.z
    proto.pose.pose.orientation.w = msg.pose.pose.orientation.w

    proto.twist.twist.linear.x = msg.twist.twist.linear.x
    proto.twist.twist.linear.y = msg.twist.twist.linear.y
    proto.twist.twist.linear.z = msg.twist.twist.linear.z

    proto.twist.twist.angular.x = msg.twist.twist.angular.x
    proto.twist.twist.angular.y = msg.twist.twist.angular.y
    proto.twist.twist.angular.z = msg.twist.twist.angular.z


def timerCallback(event):

    #Check if all msgs were received
    if rcv_left_image == False or rcv_right_image == False or rcv_nav_sat_fix == False or rcv_odometry == False or rcv_point_cloud == False:
        print("Did not receive all required ros messages. Cannot publish the zmq message. Make sure messages are being published at least at 1Hz on topics:\n/odom " + str(rcv_odometry) + "\n/stereo/left/image_raw " + str(rcv_left_image) + "\n/mavros/global_position/raw/fix " + str(rcv_nav_sat_fix)+ "\n/stereo/points2 " + str(rcv_point_cloud) + "\n/stereo/right/image_raw " + str(rcv_right_image))
        return None

    #Start preparing message to send
    start_time = time.time()

    sd = SEVData_pb2.SEVData()

    # Copying left image
    cvImage2Proto(cv_left_image, left_image_header, sd.left_image)

    # Copying right image
    cvImage2Proto(cv_right_image, right_image_header, sd.right_image)

    # Copying the point cloud
    pointCloudMsg2Proto(point_cloud2_msg, sd.point_cloud)

    # Copying NavSatFix
    navSatFixMsg2Proto(nav_sat_fix_msg, sd.nav_sat_fix)

    #Copying Odometry
    odometryMsg2Proto(odometry_msg, sd.odometry)

    #Copying the header
    sd.header.frame_id = ""
    t = rospy.Time.now()
    sd.header.stamp.seconds = t.secs
    sd.header.stamp.nanos = t.nsecs

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

    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, navSatFixReceivedCallback)

    rospy.Subscriber("/odom", Odometry, odometryReceivedCallback)


    #cv2.namedWindow("Left Camera")
    #cv2.namedWindow("Right Camera")

    #TODO This wait is to try to get at least one message of each time before publishing the message. Its a blind wait so sometimes it does not work. In the future, the condition above should be checked before sending
    rospy.Timer(rospy.Duration(0.1), timerCallback)

    #Spin infinetely
    rospy.spin()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

