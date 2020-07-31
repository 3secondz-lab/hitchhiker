#!/usr/bin/python3
"""
    Openpilot-ROS message converter

    Jinsun Park
    (zzangjinsun@3secondz.com)
"""

# Openpilot
import cereal.messaging as messaging
from cereal import log, car
from capnp.lib import capnp
import zmq

import time
import os
os.environ["ZMQ"] = "1"


if __name__ == '__main__':

    submaster = messaging.SubMaster(['frame'])

    while True:
        submaster.update()

        for k, v in submaster.updated.items():
            print('{} : {}'.format(k, v))

        time.sleep(0.5)

    print('Test finished')
