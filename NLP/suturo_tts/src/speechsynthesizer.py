#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import pyttsx

speak_rate = rospy.get_param('speak_rate', 125)
speak_volume = rospy.get_param('speak_volume', 10)
speak_voice = rospy.get_param('speak_voice', 0)


def callback(data):
    engine = pyttsx.init()
    engine.setProperty('rate', speak_rate)
    engine.setProperty('volume', speak_volume)
    voices = engine.getProperty('voices')  # getting details of current voice
    engine.setProperty('voice', voices[speak_voice].id)
    engine.say(data.data)
    engine.runAndWait()
    rospy.loginfo("I said %s", data.data)


def listener():
    rospy.init_node('TextToSpeechSynthesizer')
    rospy.Subscriber("tts", String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()