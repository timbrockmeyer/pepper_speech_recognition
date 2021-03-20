#!/usr/bin/env python
import rospy
import subprocess

from std_msgs.msg import Int32

def microphone_volume_cb(data):
    try:
        volume = data.data
        callProcess = subprocess.Popen(['amixer', 'sset', 'Capture', str(volume)], shell=False)
    except:
        rospy.loginfo("microphone volume request failed")

def set_microphone_volume():
    rospy.init_node('set_microphone_volume', anonymous=True)
    rospy.Subscriber('microphone_volume', Int32, microphone_volume_cb)
    print "Ready to set mic volume by publishing an integer from 1-100 to /microphone_volume."
    print "Usually you only want to mute or unmute, i.e. set the Volume to 0 or 100."
    print "Examples from the console:"
    print "rostopic pub --once /microphone_volume std_msgs/Int32 100"
    print "rostopic pub --once /microphone_volume std_msgs/Int32 0"

    rospy.spin()

if __name__ == "__main__":
    set_microphone_volume()
