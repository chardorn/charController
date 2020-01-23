#! /usr/bin/env python
import rospy
from ac_msgs.msg import drive_params
from std_msgs.msg import Bool
import time
from sensor_msgs.msg import LaserScan

def lidar_callback(distances):
    num_scans = len(distances)
    max_distance = 0.0
    for sample in distances:
        if sample > max_distance:
            max_distance = sample
    print("Max distance:")
    print(max_distance)



rospy.init_node("BugControl")
DriveParamPublisher = rospy.Publisher("drive_parameters", drive_params, queue_size=10)
EStopPublisher = rospy.Publisher("eStop", Bool, queue_size=10)
rospy.Subscriber('scan', LaserScan, lidar_callback)
rospy.spin()

msg = drive_params()
while(False):

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
