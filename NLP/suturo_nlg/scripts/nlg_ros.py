#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import String
from suturo_nlg.msg import MeaningRepresentation, KeyValuePair

import subprocess
import os

import time

import zmq
from tinyrpc import RPCClient
from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.zmq import ZmqClientTransport

from tmc_msgs.msg import Voice, TalkRequestGoal, TalkRequestActionGoal, TalkRequestAction
import actionlib 

print("Starting up ...")

ctx = zmq.Context()

rpc_client = RPCClient(
    JSONRPCProtocol(),
    ZmqClientTransport.create(ctx, 'tcp://127.0.0.1:5002')
)

str_server = rpc_client.get_proxy()

rospy.init_node('talker', anonymous=True)
pub = rospy.Publisher('debug_textgen', String, queue_size=1)


def callback(data):
    dataPie = {}
    for rv in data.role_values:
        isNumber = (rv.key.rfind("Number") != -1)
        if isNumber:
            dataPie[rv.key] = int(rv.value)
        else:
            dataPie[rv.key] = rv.value
    print(dataPie)
    message = str_server.generate_text(dataPie)
    pub.publish(message)
    client = actionlib.SimpleActionClient('talk_request_action', TalkRequestAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print("Action available")
    # Creates a goal to send to the action server.
    goal = TalkRequestGoal(data=Voice(language=1, sentence=message))
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    return

rospack = rospkg.RosPack()
wd = rospack.get_path('suturo_nlg')

rospy.Subscriber("nlg_requests", MeaningRepresentation, callback)

#for x in ["reject", "task", "waiting", "failed", "finished", "complications"]:
#    pub.publish(str_server.generate_text({"action": "bring", "patient": "pills", "patientNumber":300, "destination": "cabinet"}, x))

#subprocess.call('python3 nlg.py', cwd=wd, shell=True)
#subprocess.Popen(["python3", os.path.join(wd, "nlg.py")])

#time.sleep(5)
#for x in ["reject", "task", "waiting", "failed", "finished", "complications"]:
#    print("    Trying out", {"action": "bring", "patient": "pills", "patientNumber":300, "destination": "cabinet"}, x)
#    pub.publish(str_server.generate_text({"action": "bring", "patient": "pills", "patientNumber":300, "destination": "cabinet", "decision": x}))

message = MeaningRepresentation()

message.role_values.append(KeyValuePair(key = "action", value = "bring"))
message.role_values.append(KeyValuePair(key = "patient", value = "pills"))
message.role_values.append(KeyValuePair(key = "patientNumber", value = "300"))
message.role_values.append(KeyValuePair(key = "destination", value = "cabinet"))
message.role_values.append(KeyValuePair(key = "decision", value = "task"))

callback(message)


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()

