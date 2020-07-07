#!/usr/bin/python3
"""
    Openpilot-ROS message converter

    Jinsun Park
    (zzangjinsun@3secondz.com)
"""


# ROS
import rospy

# Openpilot
import cereal.messaging as messaging
from cereal import log


class MessageConverter:
    def __init__(self, list_service):
        '''
        Convert Openpilot services to ROS messages

        :param list_service: List of Openpilot services to subscribe
        '''

        pm = messaging.PubMaster(['sensorEvents'])
        dat1 = messaging.new_message('sensorEvents', size=1)


        d1 = dat.to_bytes()
        d2 = log.Event.from_bytes(d1)


        sm = messaging.SubMaster(['sensorEvents', 'frame'])

        d1 = sm.data.to_bytes()




        print(service_list)











if __name__ == '__main__':
    mc = MessageConverter()
