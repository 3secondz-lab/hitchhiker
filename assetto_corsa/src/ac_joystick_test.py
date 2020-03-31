#!/usr/bin/python3
"""
    Assetto Corsa Publisher

    Jinsun Park
    (zzangjinsun@3secondz.com)
"""


import rospy
from assetto_corsa.msg import ACRaw
from translate import *
from std_msgs.msg import String

import logging
from ac_parseai import *

debug_logger = logging.getLogger('dev')
debug_logger.setLevel(logging.INFO)
debug_handler = logging.StreamHandler()
debug_handler.setFormatter(logging.Formatter('[%(asctime)s] %(name)s-%(levelname)s: %(message)s'))
debug_logger.addHandler(debug_handler)
debug_logger.info("ac_pub.py")

aipath = ai('/home/rnd/.steam/steam/steamapps/common/assettocorsa', 'imola', debug_logger)

list_plane_name = ['center_x', 'center_y', 'left_x', 'left_y', 'right_x', 'right_y',
                   'radius', 'psie', 'cte']
list_cam_name = ['center_x', 'center_y', 'left_x', 'left_y', 'right_x', 'right_y']



if __name__ == '__main__':
    # Parameters
    name_node = 'ac_pub'
    freq_pub = rospy.get_param('/{}/freq_pub'.format(name_node), 1000)
    print('freq_pub : {}'.format(freq_pub))

    rospy.init_node(name_node)

    host = rospy.get_param('/{}/host'.format(name_node), 'localhost')
    port = rospy.get_param('/{}/port'.format(name_node), 12345)

    # Connect
    target = (host, port)
    print('Target : {}:{}'.format(host, port))

    if host == 'localhost':
        ip_bind = 'localhost'
    else:
        ip_bind = '0.0.0.0'

    s = socket(AF_INET, SOCK_STREAM)
    # s.bind((ip_bind, 0))
    s.bind(('', port))
    print('Local socket : {}:{}'.format(ip_bind, s.getsockname()[1]))
    s.listen()

    client_socket, addr = s.accept()
    print('accepted')

    rate = rospy.Rate(freq_pub)

    while not rospy.is_shutdown():
        try:
            data = client_socket.recv(65535)

            data_dec = data.decode()

            print(data_dec)

            rate.sleep()
        except KeyboardInterrupt:
            print('Finishing...')
            dismiss(s, target)
            break

    print('{} node finished.'.format(name_node))
