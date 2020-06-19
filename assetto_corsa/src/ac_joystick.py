#!/usr/bin/python3
"""
    Assetto Corsa Joystick Controller

    Jinsun Park
    (zzangjinsun@3secondz.com)
"""

import rospy
from std_msgs.msg import Float64MultiArray
from socket import *
import ctypes
import time

socket_global = 0


class JoystickPayload(ctypes.Structure):
    _fields_ = [
        ('steering', ctypes.c_float),
        ('accel', ctypes.c_float),
        ('brake', ctypes.c_float)
    ]


def connect_socket():
    global socket_global

    socket_global = socket(AF_INET, SOCK_STREAM)

def callback(data: Float64MultiArray):
    global socket_global

    v1 = data.data[0]
    v2 = data.data[1]
    v3 = data.data[2]

    payload = JoystickPayload(v1, v2, v3)

    # data = '{},{},{}'.format(v1, v2, v3)

    try:
        socket_global.send(bytes(payload))
        # print("Data : {}, {}, {}".format(v1, v2, v3))
    except (ConnectionRefusedError, BrokenPipeError):
        socket_global = socket(AF_INET, SOCK_STREAM)
        try:
            if socket_global.connect_ex(target) != 0:
                return
            else:
                print('Reconnected')
        except ConnectionRefusedError:
            return
        socket_global.send(bytes(payload))
        # print("Data : {}, {}, {}".format(v1, v2, v3))



if __name__ == '__main__':
    # Parameters
    name_node = 'ac_joystick'
    freq_sub = rospy.get_param('{}/freq_sub'.format(name_node), 200)
    topic_name = rospy.get_param('{}/topic_name'.format(name_node), '/matlab_controller')
    print('freq_sub : {}'.format(freq_sub))

    rospy.init_node(name_node)

    host = rospy.get_param('/{}/host'.format(name_node), 'localhost')
    port = rospy.get_param('/{}/port'.format(name_node), 13240)

    # Connect
    target = (host, port)
    print('Target : {}:{}'.format(host, port))

    if host == 'localhost':
        ip_bind = 'localhost'
    else:
        ip_bind = '0.0.0.0'

    while True:
        if rospy.is_shutdown():
            print('{} node finished.'.format(name_node))
            sys.exit(0)

        socket_global = socket(AF_INET, SOCK_STREAM)
        if socket_global.connect_ex(target) == 0:
            break
        else:
            print('Waiting...')
            time.sleep(1)
            continue

    print('USB/IP Server connected')


    # print('Local socket : {}:{}'.format(ip_bind, socket_global.getsockname()[1]))

    rate = rospy.Rate(freq_sub)
    sub = rospy.Subscriber(topic_name, Float64MultiArray, callback, queue_size=1)

    rospy.spin()

    print('{} node finished.'.format(name_node))
