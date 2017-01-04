#!/usr/bin/env python

#imports
import zmq
import time
import numpy as np

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2, PointField, NavSatFix
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
import copy
import SEVData_pb2

#--------------------------
# Global Vars
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

#--------------------------
# Start of main
#--------------------------
def main(args):

    #prepare the listener
    global socket
    socket.connect("tcp://" + ip + ":" + port)
    socket.setsockopt_string(zmq.SUBSCRIBE, topic )

    #setup some opencv windows
    cv2.namedWindow("Listener Left Camera")
    cv2.namedWindow("Listener Right Camera")

    #Start the main loop
    while True:

        message_in = socket.recv(10*1024)
        #For some reason I have to make a deep copy of the message. Otherwise, when, the second time I received the message I got a truncated message error. This solves it so I did not worry about it anymore.
        message = copy.deepcopy(message_in)
        
        #Deserialization or unmarshalling
        #We use message[4:] because we know the first four bytes are
        #"777 " and we only want to give the data to the parser (not the topic name
        sd.ParseFromString(message[4:])
        
        #Getting the left image
        cv_left_image = np.zeros((sd.left_image.height, sd.left_image.width, 3), np.uint8)
        cv_left_image.data = sd.left_image.data
        
        #Getting the right image
        cv_right_image = np.zeros((sd.right_image.height, sd.right_image.width, 3), np.uint8)
        cv_right_image.data = sd.right_image.data

        #Getting the point cloud
        point_cloud_msg = PointCloud2()
        point_cloud_msg.height = sd.point_cloud.height
        point_cloud_msg.width = sd.point_cloud.width

        for field in sd.point_cloud.fields:
            point_field = PointField()
            point_field.name = field.name
            point_field.offset = field.offset
            point_field.datatype = field.datatype
            point_field.count = field.count

            point_cloud_msg.fields.append(point_field)

        point_cloud_msg.is_bigendian = sd.point_cloud.is_bigendian
        point_cloud_msg.is_bigendian = sd.point_cloud.is_bigendian
        point_cloud_msg.point_step = sd.point_cloud.point_step
        point_cloud_msg.row_step = sd.point_cloud.row_step
        point_cloud_msg.data = sd.point_cloud.data

        #Getting the NavSatFix
        nav_sat_fix = NavSatFix()
        nav_sat_fix.latitude = sd.nav_sat_fix.latitude
        nav_sat_fix.longitude = sd.nav_sat_fix.longitude
        nav_sat_fix.altitude = sd.nav_sat_fix.altitude
        nav_sat_fix.status.status = sd.nav_sat_fix.status.status
        nav_sat_fix.status.service = sd.nav_sat_fix.status.service

        #Getting the Odometry
        odometry = Odometry()
        #and then we can copy every field ... no need in this example


        #---------------------------------
        #Visualizing and /or printing the received data
        #---------------------------------
        print("Received new message with stamp:\n" + str(sd.header.stamp))
    
        print("First 10 points x,y,z and rgb (packed in float32) values (just for debug)")
        count = 0
        for p in pc2.read_points(point_cloud_msg, field_names = ("x", "y", "z", "rgb"), skip_nans=True):
            print " x : %f  y: %f  z: %f rgb: %f" %(p[0],p[1],p[2],p[3])
            count = count + 1
            if count > 10:
                break

        print("GPS data:")
        print("Latitude =" + str(nav_sat_fix.latitude))
        print("Longitude =" + str(nav_sat_fix.longitude))
        print("Altitude =" + str(nav_sat_fix.altitude))
        print("Status =" + str(nav_sat_fix.status.status))
        print("Service =" + str(nav_sat_fix.status.service))

        print("Odometry data (only position for example):")
        print("odom.pose.pose.position.x =" + str(sd.odometry.pose.pose.position.x))
        print("odom.pose.pose.position.y =" + str(sd.odometry.pose.pose.position.y))
        print("odom.pose.pose.position.z =" + str(sd.odometry.pose.pose.position.z))

        cv2.imshow("Listener Left Camera", cv_left_image)
        cv2.imshow("Listener Right Camera", cv_right_image)
        cv2.waitKey(30)



if __name__ == '__main__':
    main(sys.argv)

