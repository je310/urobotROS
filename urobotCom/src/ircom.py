#!/usr/bin/env python


import rospy


import serial
import sys
import numpy

#from std_msgs.msg import String
#from std_msgs.msg import Float32
#from std_msgs.msg import Int32
from collections import namedtuple
from struct import *




from collections import namedtuple
import struct
import time
import roslib
import rospy
from std_msgs.msg import String
import os.path
from geometry_msgs.msg import Twist
from threading import Timer
def generateChecksum(header,robots,type,msgs):
    sum = header + header + robots + (255 - robots) + type
    for x in range(0, robots):
        sum = sum + msgs[x]

    sum = sum % 256
    return sum

def recieveMsg():
    print 'please implement a function that recieves messages'
    return 666

def timeToWait(wantedFreq,robots,bandwidth):
    byteTotal = 6 + robots
    time = (byteTotal * 8.0) / bandwidth
    if (time > (1.0/wantedFreq)):
        print 'there will be a bottle neck on IR comms channel'
    return time +0.05




rob0 = 7 + 16*7
rob1 = 7 + 16*7
rob2 = 7 + 16*7
rob3 = 7 + 16*7

def rob0CB(data):
    global rob0
    rot = int(15*(data.angular.z + 1)/2)
    lin = int(15*(data.linear.x + 1)/2)
    rob0 = lin +16* rot

def rob1CB(data):
    global rob1
    rot = int(15*(data.angular.z + 1)/2)
    lin = int(15*(data.linear.x + 1)/2)
    rob1 = lin +16* rot

def rob2CB(data):
    global rob2
    rot = int(15*(data.angular.z + 1)/2)
    lin = int(15*(data.linear.x + 1)/2)
    rob2 = lin +16* rot

def rob3CB(data):
    global rob3
    rot = int(15*(data.angular.z + 1)/2)
    lin = int(15*(data.linear.x + 1)/2)
    rob3 = lin +16* rot
typeTemp = 6;
robotsTemp = 4;
def fightCB(data):
    global typeTemp, robotsTemp
    if data.data == 'fight':
        typeTemp = 9
        robotsTemp = 4



#    print rot, lin



def talker():
    global typeTemp, robotsTemp
    global rob0,rob1,rob2,rob3
    global rob0s
    #fake data for prototyping
    robots = 4
    header = 0xAC
    type = 6
    msgs = [rob0,rob1,rob2,rob3]
    checksum = 0
    wantedFreq =5
    bandwidth = 2200
    outward = 1


    rospy.init_node('urobotMaster', anonymous=True)
    rospy.Subscriber("rob0", Twist, rob0CB)
    rospy.Subscriber("rob1", Twist, rob1CB)
    rospy.Subscriber("rob2", Twist, rob2CB)
    rospy.Subscriber("rob3", Twist, rob3CB)
    rospy.Subscriber("fight", String, fightCB)
    serdev = '/dev/ttyACM0'
    master = serial.Serial(
    port=serdev,
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout = 0.1,
    rtscts=0,
     write_timeout=0.01
    )

    while not rospy.is_shutdown():
        type = typeTemp
        robots = robotsTemp
        msgs = [rob0,rob1,rob2,rob3]
        #print msgs

        #send header. 2 bytes
        master.write(chr(header))
        master.write(chr(header))

        #send robot number
        master.write(chr(robots))
        master.write(chr(255-robots))

        #send type (move, report batt voltage etc)
        master.write(chr(type))

        #Send the command value for each robot
        for x in range(0, robots):
            master.write(chr(msgs[x]))

        #send the checksum
        checksum = generateChecksum(header,robots,type,msgs)
        master.write(chr(checksum))

        #if special fight command may need to wait a while, wait for confirmation from master.
        loopcount = 0
        if type == 9:
            first = 0
            second =1
            third = 3
            confirmed = 0
            confState = 0
            #time.sleep(4)
            master.flushInput()
            while confirmed ==0 and loopcount <300:
                loopcount= loopcount +1
                value = master.read()
                if confState == 0 and len(value) >0:
                    print ord(value)
                    if ord(value) == 0xAC:
                        print 'got ac'
                        confState = 1
                        continue
                if confState == 1:
                    if ord(value) == 0xCA:
                        confState = 2
                        continue
                    else:
                        confState = 0
                if confState == 2:
                    first = ord(value)
                    confState = 3
                    continue
                if confState == 3:
                    second = ord(value)
                    confState = 4
                    continue
                if confState == 4:
                    third = ord(value)
                    confirmed = 1
                    if (second == first and second == third):
                         robots = first
                         print 'ROBOTS'
                         print robots
                         master.flushInput()
                         typeTemp = 6
                         confirmed = 1
                         robotsTemp = robots
                    else:
                        print 'there has been an error in the fight for ID proceedure'
        typeTemp = 6
        #recieve confirmation of successful recieve from masterBot
#        confirmation = 'n'
#        confirmation = master.read()
#        if(confirmation != 'y'):
#            print 'error in sending to master bot'
#            time.sleep(0.1)
       # master.reset_input_buffer()
        if(outward !=1):
            messageReceived = recieveMsg()

        #master.flush()
        #calculate time to fill the IR bandwidth, could use 1/wantedFreq if want well defined Hz to ros.
        waitTime = timeToWait(wantedFreq,robots, bandwidth)
#        print waitTime
#        master.close()
        time.sleep(waitTime)






if __name__ == '__main__':
    talker()
