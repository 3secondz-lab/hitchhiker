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
import sys
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


def generate_msg_from_data(data_udp: payload_t, preview: dict):
    m = ACRaw()

    m.header.stamp = rospy.Time.now()

    # m.identifier = String(data=data_udp.identifier[0])
    m.size = data_udp.size
    m.speed_Kmh = data_udp.speed_Kmh
    m.speed_Mph = data_udp.speed_Mph
    m.speed_Ms = data_udp.speed_Ms
    m.isAbsEnabled = data_udp.isAbsEnabled
    m.isAbsInAction = data_udp.isAbsInAction
    m.isTcInAction = data_udp.isTcInAction
    m.isTcEnabled = data_udp.isTcEnabled
    m.isInPit = data_udp.isInPit
    m.isEngineLimiterOn = data_udp.isEngineLimiterOn
    m.accG_vertical = data_udp.accG_vertical
    m.accG_horizontal = data_udp.accG_horizontal
    m.accG_frontal = data_udp.accG_frontal
    m.lapTime = data_udp.lapTime
    m.lastLap = data_udp.lastLap
    m.bestLap = data_udp.bestLap
    m.lapCount = data_udp.lapCount
    m.gas = data_udp.gas
    m.brake = data_udp.brake
    m.clutch = data_udp.clutch
    m.engineRPM = data_udp.engineRPM
    m.steer = data_udp.steer
    m.gear = data_udp.gear
    m.cgHeight = data_udp.cgHeight

    for k in range(0, 4):
        m.wheelAngularSpeed[k] = data_udp.wheelAngularSpeed[k]
        m.slipAngle[k] = data_udp.slipAngle[k]
        m.slipAngle_ContactPatch[k] = data_udp.slipAngle_ContactPatch[k]
        m.slipRatio[k] = data_udp.slipRatio[k]
        m.tyreSlip[k] = data_udp.tyreSlip[k]
        m.ndSlip[k] = data_udp.ndSlip[k]
        m.load[k] = data_udp.load[k]
        m.Dy[k] = data_udp.Dy[k]
        m.Mz[k] = data_udp.Mz[k]
        m.tyreDirtyLevel[k] = data_udp.tyreDirtyLevel[k]
        m.camberRAD[k] = data_udp.camberRAD[k]
        m.tyreRadius[k] = data_udp.tyreRadius[k]
        m.tyreLoadedRadius[k] = data_udp.tyreLoadedRadius[k]
        m.suspensionHeight[k] = data_udp.suspensionHeight[k]
    m.carPositionNormalized = data_udp.carPositionNormalized
    m.carSlope = data_udp.carSlope

    for k in range(0, 3):
        m.carCoordinates[k] = data_udp.carCoordinates[k]

    if preview is not None:
        # Check validity
        for key in list_plane_name:
            if key in preview['plane'].keys():
                if type(preview['plane'][key]) == list and \
                        len(preview['plane'][key]) == 0:
                    print('key {} length 0'.format(key))
                    return m
            else:
                print('key {} not found'.format(key))
                return m

        for key in list_cam_name:
            if key in preview['cam'].keys():
                if type(preview['plane'][key]) == list and \
                        len(preview['cam'][key]) == 0:
                    print('key {} length 0'.format(key))
                    return m
            else:
                print('key {} not found'.format(key))
                return m

        # plane = [center_x, center_y, left_x, left_y, right_x, right_y, radius, psie, cte]
        m.plane_center_x = np.array(preview['plane']['center_x'], dtype=np.float64)
        m.plane_center_y = np.array(preview['plane']['center_y'], dtype=np.float64)
        m.plane_left_x = np.array(preview['plane']['left_x'], dtype=np.float64)
        m.plane_left_y = np.array(preview['plane']['left_y'], dtype=np.float64)
        m.plane_right_x = np.array(preview['plane']['right_x'], dtype=np.float64)
        m.plane_right_y = np.array(preview['plane']['right_y'], dtype=np.float64)
        m.plane_radius = np.array(preview['plane']['radius'], dtype=np.float64)
        m.plane_psie = preview['plane']['psie']
        m.plane_cte = preview['plane']['cte']

        # cam = [center_x, center_y, left_x, left_y, right_x, right_y]
        m.cam_center_x = np.array(preview['cam']['center_x'], dtype=np.float64)
        m.cam_center_y = np.array(preview['cam']['center_y'], dtype=np.float64)
        m.cam_left_x = np.array(preview['cam']['left_x'], dtype=np.float64)
        m.cam_left_y = np.array(preview['cam']['left_y'], dtype=np.float64)
        m.cam_right_x = np.array(preview['cam']['right_x'], dtype=np.float64)
        m.cam_right_y = np.array(preview['cam']['right_y'], dtype=np.float64)

    return m


if __name__ == '__main__':
    # Parameters
    name_node = 'ac_pub'
    freq_pub = rospy.get_param('/{}/freq_pub'.format(name_node), 500)
    print('freq_pub : {}'.format(freq_pub))

    rospy.init_node(name_node)

    host = rospy.get_param('/{}/host'.format(name_node), 'localhost')
    port = rospy.get_param('/{}/port'.format(name_node), 9996)

    # Connect
    target = (host, port)
    print('Target : {}:{}'.format(host, port))

    if host == 'localhost':
        ip_bind = 'localhost'
    else:
        ip_bind = '0.0.0.0'

    s = socket(AF_INET, SOCK_DGRAM)
    s.bind((ip_bind, 0))
    print('Local socket : {}:{}'.format(ip_bind, s.getsockname()[1]))

    print('Waiting for {} listening...'.format(port))

    while True:
        if rospy.is_shutdown():
            print('{} node finished.'.format(name_node))
            sys.exit(0)
        if not target_on_listening(s, target):
            print('Waiting...')
            time.sleep(0.5)
        else:
            break
    print('Target on listening.')

    handshake(s, target)

    rate = rospy.Rate(freq_pub)
    pub = rospy.Publisher('/{}/ACRaw'.format(name_node), ACRaw, queue_size=1)

    while not rospy.is_shutdown():
        try:
            data, _ = s.recvfrom(1024)
            if len(data) != 328:
                print('Inconsistent data length : {}'.format(len(data)))
                continue
            payload = payload_t.from_buffer_copy(data)

            # Get additional data
            aipath.recv_telemetry(payload)

            # Convert data to msg
            m = generate_msg_from_data(payload, aipath.preview)

            # print(aipath.preview['plane'])

            pub.publish(m)

            rate.sleep()
        except KeyboardInterrupt:
            print('Finishing...')
            dismiss(s, target)
            break

    dismiss(s, target)
    print('{} node finished.'.format(name_node))
