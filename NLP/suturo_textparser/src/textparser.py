#!/usr/bin/env python
import rospy
import rospkg
from tmc_rosjulius_msgs.msg import RecognitionResult
from nlp_msgs.msg import StaticCommand


def callback(data):
    sentence = findFirst(data)
    # if not sentence == "" and not isStatic(sentence):
    isStatic(sentence)
    # TODO make sling call



def isStatic(sentence):
    if isStop(sentence):
        command = StaticCommand()
        command.command = StaticCommand.STOP
        pub.publish(command)
        return True
    elif isStart(sentence):
        command = StaticCommand()
        command.command = StaticCommand.START
        pub.publish(command)
        return True
    elif isContinue(sentence):
        command = StaticCommand()
        command.command = StaticCommand.CONTINUE
        pub.publish(command)
        return True

    return False


def isStop(sentence):
    return isInFile(sentence, wd + '/commands/stop.txt')


def isStart(sentence):
    return isInFile(sentence, wd + '/commands/start.txt')

def isContinue(sentence):
    return isInFile(sentence, wd + '/commands/continue.txt')


def isInFile(sentence, filepath):
    with open(filepath) as f:
        if sentence in f.read():
            return True
        else:
            return False


def findFirst(data):
    mostliklySentence = data.sentences[0]
    score1 = data.scores[0]

    if len(data.scores) == 1 and score1 > 0.80:
        return mostliklySentence
    score2 = data.scores[1]
    if score1 > 0.93 and score1 - score2 > 0.1:
        return mostliklySentence
    return ""


if __name__ == '__main__':
    try:
        rospack = rospkg.RosPack()
        wd = rospack.get_path('suturo_textparser')
        rospy.init_node('textparser', anonymous=False)
        pub = rospy.Publisher('hard_commands', StaticCommand, queue_size=10)
        rospy.Subscriber('input', RecognitionResult, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
