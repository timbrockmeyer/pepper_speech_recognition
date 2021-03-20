#!/usr/bin/env python

#receives naoqi ALTextToSpeech/Status via socket from the corresponding bridge python script
#and forwards it into rostopic /pepper_speech_status
#so we can mute the usb microphone when pepper is speaking
#since we can not run naoqi and rospy with the same python version

import sys
import time
import rospy

from std_msgs.msg import String

import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #avoid bugs with closed sockets
s.bind((socket.gethostname(), 1619))
s.listen(5)
print("Socket Receiver initialized!")

pub = rospy.Publisher("pepper_speech_status", String, queue_size=10)
rospy.init_node('speech_status_publisher', anonymous=True)

while True:
    clientsocket, address = s.accept()
    data = clientsocket.recv(1024)
    rospy.loginfo("Got data via socket: " + str(data))
    if data == "enqueued" or data == "started" or data =="done":
        rospy.loginfo("Sent out data to rostopic /pepper_speech_status: " + str(data))
        pub.publish(data)
    clientsocket.send("ok")
    clientsocket.close()
