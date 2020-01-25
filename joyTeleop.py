#! /usr/bin/env python
import rospy
from ac_msgs.msg import drive_params
from std_msgs.msg import Bool
import time
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
import threading

msg = drive_params()
msg.angle = 0.5
global_scans = LaserScan()
max_index = 540
mode = 0 # 0 for manual, 1 for autonomous


def joy_callback(data):
    print("Call back!")
    global mode

    if data.buttons[1] == 1:
        mode = 1
        if not t1.isAlive():
            t1.start() #start autonomous()
    else:
        if t1.isAlive():
            stop_threads = True
            t1.join()
        autonomous = 0
        manual(data)

def manual(data):
    if data.axes[5] > 0:
        print("forward")
        if data.buttons[5] == 1.0:
            msg.velocity = 0.2
        else:
            msg.velocity = 0.1
    elif data.axes[5] < 0:
        print("backward")
        if data.buttons[5] == 1.0:
            msg.velocity = 0.2
        else:
            msg.velocity = -0.1
    else:
        msg.velocity = 0
    msg.angle = 0.5 - (data.axes[2] / 2.0)
    print(msg.angle)
    DriveParamPublisher.publish(msg)

def autonomous(stop):
    print("Called Autonomous")
    global mode
    while(mode == 1):
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
        if stop():
                break

    if t1.isAlive():
        t1.join()

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

print("Setting up... ")
t1 = threading.Thread(target=autonomous, args =(lambda : stop_threads, )

DriveParamPublisher = rospy.Publisher("drive_parameters", drive_params, queue_size=10)
EStopPublisher = rospy.Publisher("eStop", Bool, queue_size=10)
rospy.Subscriber('scan', LaserScan, lidar_callback)
rospy.Subscriber('joy', Joy, joy_callback)
rospy.init_node("JoyControl")
rospy.spin()

t1.join()
#t2.join()
