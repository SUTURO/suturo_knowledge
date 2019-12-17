#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import speech_recognition as sr

# obtain audio from the microphone
r = sr.Recognizer()


def talker():
    pub = rospy.Publisher('speechToText', String, queue_size=10)
    rospy.init_node('speechRecognizer', anonymous=True)

    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(
            source)  # listen for 1 second to calibrate the energy threshold for ambient noise levels
        while not rospy.is_shutdown():
            audio = r.listen(source)
            # recognize speech using either pocketsphinx or Google Speech Recognition
            try:
                # when using google : for testing purposes, we're just using the default API key
                #       to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
                #       instead of `r.recognize_google(audio)`
                pub_str = r.recognize_sphinx(audio) # use google instead ?
            except sr.UnknownValueError:
                pub_str = "Recognizer could not understand audio"
            except sr.RequestError as e:
                pub_str = str(e)
            rospy.loginfo(pub_str)
            pub.publish(pub_str)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
