#!/usr/bin/env python

#listens to recognized_speech via rospy and forwards it to the corresponding bridge python script
#which uses naoqi to display it on pepper's tablet and force it into pepper's dialogue system via force_input
#since we can not run naoqi and rospy with the same python version

import rospy
import socket
from std_msgs.msg import String

class SpeechSocketClient:

    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((socket.gethostname(), 1620))

    def __del__(self):
        self.s.close()

    def recognized_speech_cb(self, data):
        msg = data.data + "<br>"
        print("Sending via socket:", msg)
        self.s.send(msg)
        rospy.loginfo(rospy.get_caller_id() + 'recognized speech: %s', msg)

    def run(self):
        rospy.init_node('recognized_speech_naoqi_bridge', anonymous=True)
        rospy.Subscriber('recognized_speech', String, self.recognized_speech_cb)
        rospy.spin()

if __name__ == '__main__':
    SpeechSocketClient().run()
