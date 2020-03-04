#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import String
from nlg_msgs.msg import MeaningRepresentation

import subprocess
import os

import time

import zmq
from tinyrpc import RPCClient
from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.zmq import ZmqClientTransport

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
            dataPie[rv.key] = int(dataPie[rv.value])
        else:
            dataPie[rv.key] = dataPie[rv.value]
    message = str_server.generate_text(dataPie)
    pub.publish(message)
    return

rospack = rospkg.RosPack()
wd = rospack.get_path('nlg_msgs')

rospy.Subscriber("nlg_requests", MeaningRepresentation, callback)

#for x in ["reject", "task", "waiting", "failed", "finished", "complications"]:
#    pub.publish(str_server.generate_text({"action": "bring", "patient": "pills", "patientNumber":300, "destination": "cabinet"}, x))

#subprocess.call('python3 nlg.py', cwd=wd, shell=True)
#subprocess.Popen(["python3", os.path.join(wd, "nlg.py")])

time.sleep(5)
for x in ["reject", "task", "waiting", "failed", "finished", "complications"]:
    print("    Trying out", {"action": "bring", "patient": "pills", "patientNumber":300, "destination": "cabinet"}, x)
    pub.publish(str_server.generate_text({"action": "bring", "patient": "pills", "patientNumber":300, "destination": "cabinet"}, x))

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()

