#!/usr/bin/env python
#Python publisher using zeromq and google protocol buffers

#imports
import sys
from scipy.spatial import distance
from copy import deepcopy

import rospy
import message_filters

from nav_msgs.msg import Odometry

#--------------------------
# Classes
#--------------------------

class OdometryHandler:
    received_msg_first = False
    received_msg = False
    msg_first = 0
    msg = 0
    distance_travelled = 0

    def __init__(self):
        print("Initialized")

    def callback(self, data):

        self.msg = data
        self.received_msg = True

        if self.received_msg_first == False:
            self.msg_first = deepcopy(self.msg)
            self.received_msg_first = True
        else:
            pt0 = (self.msg_first.pose.pose.position.x, self.msg_first.pose.pose.position.y)
            pt1 = (self.msg.pose.pose.position.x, self.msg.pose.pose.position.y)
            
            self.distance_travelled = distance.euclidean(pt0, pt1)


#--------------------------
# Global Vars
#--------------------------

gps_odometry = OdometryHandler()
stereo_odometry = OdometryHandler()
time_start = 0
N = 0
accumulated_error = 0


#--------------------------
# Functions
#--------------------------

##
# @brief Called when a timer event is triggered. Will send a new message
#
# @param event the timer event
#
# @return None
def timeSyncCallback(msg_odometry_gps, msg_odometry_stereo):

    #Use these global variables
    global gps_odometry
    global stereo_odometry
    global accumulated_error
    global N

    print("Timer callback")

    gps_odometry.callback(msg_odometry_gps)
    stereo_odometry.callback(msg_odometry_stereo)

    error = abs(gps_odometry.distance_travelled - stereo_odometry.distance_travelled)
    accumulated_error += error
    N += 1

    print("gps_odometry.distance_travelled " + str(gps_odometry.distance_travelled))
    print("stereo_odometry.distance_travelled " + str(stereo_odometry.distance_travelled))

    print("Error is " + str(error) + " (average of all measurements = " + str(accumulated_error / (N+1)))





    #def callback(image, camera_info):
          ## Solve all of perception here...

          #rospy.spin()


#--------------------------
# Start of main
#--------------------------
def main(args):

    #-------------------------------------------
    # Configuration of the ros node
    #-------------------------------------------
    rospy.init_node('measure_odom_error', anonymous=True)

    time_start = rospy.Time.now()

    subscriber_gps_odometry = message_filters.Subscriber("/gps/odometry", Odometry)
    subscriber_stereo_odometry = message_filters.Subscriber("/stereo_odometer/odometry", Odometry)
    #rospy.Subscriber(self.topic, Odometry, self.callback)

    time_sync = message_filters.ApproximateTimeSynchronizer([subscriber_gps_odometry, subscriber_stereo_odometry], 10, 0.1)
    time_sync.registerCallback(timeSyncCallback)

    
    #This wait is to try to get at least one message of each time before publishing the message. Its a blind wait so sometimes it does not work. In the future, the condition above should be checked before sending
    #rospy.Timer(rospy.Duration(1), timerCallback)

    #Spin infinetely
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

