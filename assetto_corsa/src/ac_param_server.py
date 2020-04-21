#!/usr/bin/env python3

"""
    Assetto Corsa Target Param Server
    Jinsun Park
    (zzangjinsun@3secondz.com)
"""

import sys
sys.path.insert(0, "/home/rnd/catkin_ws/install/lib/python3/dist-packages")

import rospy

from dynamic_reconfigure.server import Server
from assetto_corsa.cfg import ACTargetParamsConfig


def callback(config, level):
    rospy.loginfo("Parameter reconfigured - vxInput : {vxInput}, accMax : {accMax}, axMax : {axMax}, ayMax : {ayMax}".format(**config))

    return config


if __name__ == "__main__":
    rospy.init_node("ac_param_server", anonymous=False)

    srv = Server(ACTargetParamsConfig, callback)
    rospy.spin()
