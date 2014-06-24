#!/usr/bin/env python

import rospy
import time

from std_msgs.msg import String


def kukaCallback1(data):
    global kuka1answered
    
    rospy.loginfo("kuka1:%s ",data.data)

    if data.data == "heard you":
        kuka1answered = True
        time.sleep(2)
        talkToKuka1.publish("follow me")

    if data.data == "searching":
        rospy.loginfo("kuka 1 is searching") 
    
    if data.data == "marker found":
        time.sleep(2)
        talkToKuka1.publish("stop following") 
    
    if data.data == "stopping":
        rospy.loginfo("Kuka 1 is driving to its workstation") 
    
    if data.data == "done": 
        time.sleep(2)
        talkToKuka2.publish("follow me") 

def kukaCallback2(data):
    global kuka2answered
    rospy.loginfo("kuka2: %s", data.data)

    if data.data == "heard you":
        kuka2answered = True

    if data.data == "searching":
        rospy.loginfo("kuka 2 is searching") 
    
    if data.data == "marker found":
        time.sleep(2)
        talkToKuka2.publish("stop following") 
    
    if data.data == "stopping":
        rospy.loginfo("Kuka 2 is driving to its workstation") 
    
    if data.data == "done": 
        time.sleep(2)


if __name__ == '__main__':
    rospy.init_node('pseudoScitos')


    talkToKuka1 = rospy.Publisher("scitosChatter1", String)
    talkToKuka2 = rospy.Publisher("scitosChatter2", String)
    
    global kuka1answered
    global kuka2answered
    kuka1answered = False
    kuka2answered = False

    rospy.Subscriber('kuka1Chatter', String, kukaCallback1)
    rospy.Subscriber('kuka2Chatter', String, kukaCallback2)

    while not kuka1answered or not kuka2answered:
        if not kuka1answered:
            talkToKuka1.publish("scitos here")
        if not kuka2answered:
            talkToKuka2.publish("scitos here") 
        time.sleep(1)
        rospy.is_shutdown()
    
    rospy.spin()
