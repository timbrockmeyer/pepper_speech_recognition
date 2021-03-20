#!/usr/bin/env python

#not needed for production, but in case you just want to publish pepper's speech status and nothing else

import qi
import sys
import time
import rospy

from std_msgs.msg import String

NAO_IP = '192.168.1.100'
NAO_PORT = 9559

class SpeechStatusPublisher:

    def __init__(self):
        self.pub = rospy.Publisher("pepper_speech_status", String, queue_size=10)
        rospy.init_node('speech_status_publisher', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz

        try:
            connection_url = "tcp://" + NAO_IP + ":" + str(NAO_PORT)
            app = qi.Application(["speech_status publisher", "--qi-url=" + connection_url])
        except:
            print "Failed to connect to " + NAO_IP + ":" + str(NAO_PORT)

        app.start()
        session = app.session

        self.memory = session.service("ALMemory")
        self.subscriber = self.memory.subscriber("ALTextToSpeech/Status") #optional: ("ALTextToSpeech/CurrentSentence") #CurrentWord
        self.subscriber.signal.connect(self.speech_status_publisher)


    def speech_status_publisher(self, value):
        rospy.loginfo(value[1])
        self.pub.publish(value[1])


    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == "__main__":

    pub = SpeechStatusPublisher()
    try:
        pub.run()
    except rospy.ROSInterruptException:
        pass

