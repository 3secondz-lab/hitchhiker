#!/usr/bin/python3
"""
    Assetto Corsa Subscriber

    Jinsun Park
    (zzangjinsun@3secondz.com)
"""


import rospy
from assetto_corsa.msg import ACRaw


def callback(data):
    print(data)


if __name__ == '__main__':
    # Parameters
    name_node = 'ac_sub'
    freq_sub = rospy.get_param('/{}/freq_sub'.format(name_node), 30)

    rospy.init_node(name_node)

    sub = rospy.Subscriber('/ac_pub/ACRaw', ACRaw, callback)

    rospy.spin()


