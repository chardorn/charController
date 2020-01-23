#! /usr/bin/env python
import rospy
from ac_msgs.msg import drive_params
from std_msgs.msg import Bool
import time
from sensor_msgs.msg import LaserScan

msg = drive_params()

def lidar_callback(scans):
    distances = scans.ranges
    num_scans = len(distances)
    max_distance = 0.0
    for sample in distances:
        if sample > max_distance:
            max_distance = sample
    print("Max distance:")
    print(max_distance)
    print("Center distance:")
    center_distance = distances[num_scans / 2]
    print(center_distance)
    setSpeed(center_distance)

def setSpeed(distance):
    if distance < 0.3:
        msg.velocity = 0
    else:
        msg.velocity = 0.05
    DriveParamPublisher.publish(msg)

def backUp():
    msg.velocity = -0.05
    DriveParamPublisher.publish(msg)
    time.sleep(1)
    msg.velocity = 0
    DriveParamPublisher.publish(msg)


rospy.init_node("BugControl")
DriveParamPublisher = rospy.Publisher("drive_parameters", drive_params, queue_size=10)
EStopPublisher = rospy.Publisher("eStop", Bool, queue_size=10)
rospy.Subscriber('scan', LaserScan, lidar_callback)
rospy.spin()

while(False):

    #EStopPublisher.publish(False)
    for a in range(99):
        time.sleep(0.01)
        turn =  (a / 100.0)
        msg.angle = turn

        DriveParamPublisher.publish(msg)
    a = 0
    for b in range(99):
        time.sleep(0.01)
        turn =  1 - (b / 100.0)
        msg.angle = turn
        msg.velocity = 0
    b = 0
    continue
