#!/usr/bin/env python

#listens to ALTextToSpeech/Status via naoqi and forwards its status to ROS 
#so we can mute our usb microphone when pepper is speaking
#which is running in another python script with another python version since
#we can not run naoqi and rospy with the same python version

import qi
import sys
import time
import socket

NAO_IP = '192.168.1.100'
NAO_PORT = 9559

class SpeechStatusPublisher:

    def __init__(self):
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
        print("Got via Naoqi:", value)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((socket.gethostname(), 1619))
        s.send(str(value[1]))
        msg = s.recv(1024)
        print("Sent via socket:", value[1])


    def run(self):
        print("LAUNCHED PEPPER SPEECH STATUS SOCKET NAOQI PART")
        try:
            while True:
                time.sleep(5)
        except KeyboardInterrupt:
            print "Interrupted by user, stopping subscribing to ALTextToSpeech/Status"
            #self.subscriber.unsubscribe("ALTextToSpeech/Status")
            #stop
            sys.exit(0)


if __name__ == "__main__":

    pub = SpeechStatusPublisher()
    try:
        pub.run()
    except Exception as e:
        print(e)
