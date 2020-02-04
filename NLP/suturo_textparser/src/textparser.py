#!/usr/bin/env python
import rospy
from tmc_rosjulius_msgs.msg import RecogitionResult


def callback(data):
    sentence = findFirst(data)
    if not sentence == "" and isStatic(sentence):




def isStatic(sentence):
    if isStop(sentence):
        # TODO do the stopping
        return True
    elif isStart(sentence):
        # TODO do the starting
        return True
    return False


def isStop(sentence):
    return isInFile(sentence, '../commands/stop.txt')

def isStart(sentence):
    return isInFile(sentence, '../commands/start.txt')

def isInFile(sentence, filepath):
    with open(filepath) as f:
        if sentence in f.read():
            return True
        else:
            return False


def findFirst(data):
    mostliklySentence = data.sentence[0]
    score1 = data.scores[0]
    score2 = data.scores[1]

    if data.scores.length == 1 and score1 > 0.80:
        return mostliklySentence
    if score1 > 0.93 and score1 - score2 > 0.1:
        return mostliklySentence
    return ""



def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("input", RecognitionResult, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
