#! /usr/bin/env python
import rospy
from ac_msgs.msg import drive_params
from std_msgs.msg import Bool
import time
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

msg = drive_params()
msg.angle = 0.5
global_scans = LaserScan()
max_index = 540


def joy_callback(data):
    if data.buttons[1] == 1:
        t1.start()
        t2.start()
    else:
        manual(data)

def manual(data):
    if data.axes[1] > 0:
        print("forward")
        if data.buttons[5] == 1.0:
            msg.velocity = 0.2
        else:
            msg.velocity = 0.1
    elif data.axes[1] < 0:
        print("backward")
        if data.buttons[5] == 1.0:
            msg.velocity = 0.2
        else:
            msg.velocity = -0.1
    else:
        msg.velocity = 0
    msg.angle = 0.5 - (data.axes[2] / 2.0)
    print(msg.angle)
    #msg.velocity = 0.05
    DriveParamPublisher.publish(msg)

def autonomous():
    print("Autonomous!")
    global global_scans
    print(global_scans)
    distances = global_scans.ranges
    num_scans = len(distances)
    if num_scans == 0:
        return
    #start_index = int(num_scans * 0.25)
    #end_index = int(num_scans * 0.75)
    #index = start_index
    index = 0
    max_distance = 0.0
    max_index = num_scans / 2
    while(index < num_scans - 1):
        index = index + 1
        if distances[index] > max_distance:
            max_distance = distances[index]
            max_index = index
        else:
            continue
    print("Max Index: " + str(max_index))
    #left is 0 and right is 1
    #sory, this is stuipid
    float_index = 0.1 + index - 0.1
    servo_value = index / num_scans
    print("Servo Value: " + str(servo_value))
    msg.angle = servo_value
    if distances[num_scans / 2] < 0.5:
        msg.velocity = 0.0
    else:
        msg.velocity = 0.05
    DriveParamPublisher.publish(msg)

def lidar_callback(scans):
    global global_scans
    global_scans= scans

def lidar_callback_2(scans):
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

t1 = threading.Thread(target=autonomous, args=(10,))
t2 = threading.Thread(target= , args=(10,))

DriveParamPublisher = rospy.Publisher("drive_parameters", drive_params, queue_size=10)
EStopPublisher = rospy.Publisher("eStop", Bool, queue_size=10)
rospy.Subscriber('scan', LaserScan, lidar_callback)
rospy.Subscriber('joy', Joy, joy_callback)
rospy.init_node("JoyControl")
rospy.spin()





t1.join()
t2.join()
