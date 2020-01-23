#! /usr/bin/env python
import rospy
from ac_msgs.msg import drive_params
from std_msgs.msg import Bool
import time
from sensor_msgs.msg import LaserScan

rospy.init_node("BugControl")
DriveParamPublisher = rospy.Publisher("drive_parameters", drive_params, queue_size=10)
EStopPublisher = rospy.Publisher("eStop", Bool, queue_size=10)
rospy.Subscriber('scan', LaserScan, self.lidar_callback)
rospy.spin()

def lidar_callback(self, distances):
    num_scans = len(distances)
    max_distance = 0.0
    for sample in distances:
        if sample > max_distance:
            max_distance = sample
    print("Max distance:")
    print(max_distance)

msg = drive_params()
while(1):

    EStopPublisher.publish(False)

    for a in range 100:
        time.sleep(0.01)
        turn =  (a / 100)
        msg.angle = turn
        msg.velocity = 0
        DriveParamPublisher.publish(msg)
    for b in range 100:
        time.sleep(0.01)
        turn =  1 - (b / 100)
        msg.angle = turn
        msg.velocity = 0
        DriveParamPublisher.publish(msg)

    for c in range 100:
        time.sleep(0.01)
        turn =  - (a / 100)
        msg.angle = turn
        msg.velocity = 0
        DriveParamPublisher.publish(msg)

    for c in range 100:
        time.sleep(0.01)
        turn =  -1 + (a / 100)
        msg.angle = turn
        msg.velocity = 0
        DriveParamPublisher.publish(msg)
