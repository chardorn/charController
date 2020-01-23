#! /usr/bin/env python
import rospy
from ac_msgs.msg import drive_params
from std_msgs.msg import Bool
import time

rospy.init_node("BugControl")
DriveParamPublisher = rospy.Publisher("drive_parameters", drive_params, queue_size=10)
EStopPublisher = rospy.Publisher("eStop", Bool, queue_size=10)

msg = drive_params()
while(1):
    #EStopPublisher.publish(False)

    for a in range(99):
        time.sleep(0.01)
        turn =  (a / 100.0)
        msg.angle = turn
        msg.velocity = 0
        DriveParamPublisher.publish(msg)
    a = 0
    for b in range(99):
        time.sleep(0.01)
        turn =  1 - (b / 100.0)
        msg.angle = turn
        msg.velocity = 0
        DriveParamPublisher.publish(msg)
    b = 0
    continue
