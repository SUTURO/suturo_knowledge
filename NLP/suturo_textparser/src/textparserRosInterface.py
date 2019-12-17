#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import String

import subprocess
import os

import zmq
from tinyrpc import RPCClient
from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.zmq import ZmqClientTransport


ctx = zmq.Context()

rpc_client = RPCClient(
    JSONRPCProtocol(),
    ZmqClientTransport.create(ctx, 'tcp://127.0.0.1:5001')
)

str_server = rpc_client.get_proxy()


def callback(data):
    input_str = data.data

    input_array = input_str.split('\\')

    for in_str in input_array:
        parsed = str(str_server.semantic_parse(in_str))
        if isinstance(parsed, str):
            pub.publish(parsed)
        else:
            pub.publish('Didn\'t get a string from the parser')


if __name__ == '__main__':
    print('called main')
    rospack = rospkg.RosPack()
    wd = rospack.get_path('suturo_textparser')
    print(wd)
    try:
        rospy.loginfo('trying')
        # Start the publisher node
        rospy.init_node('tpri', anonymous=True)
        pub = rospy.Publisher('tp_output', String, queue_size=10)
        # Start the subscriber node
        rospy.Subscriber('tp_input', String, callback)
        subprocess.call('python3 src/textparser.py ' + wd + '/src', cwd=wd, shell=True)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
