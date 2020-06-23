#!/usr/bin/python3
"""
    Assetto Corsa Publisher

    Jinsun Park
    (zzangjinsun@3secondz.com)
"""

import rospy
from socket import *
from assetto_corsa.msg import ACRaw
from translate import *
from std_msgs.msg import String, MultiArrayDimension

from functools import wraps
import errno
import os
import signal
from contextlib import suppress

import logging
import sys
from ac_parseai import *

debug_logger = logging.getLogger('dev')
debug_logger.setLevel(logging.INFO)
debug_handler = logging.StreamHandler()
debug_handler.setFormatter(logging.Formatter('[%(asctime)s] %(name)s-%(levelname)s: %(message)s'))
debug_logger.addHandler(debug_handler)
debug_logger.info("ac_pub.py")

list_plane_name = ['center_x', 'center_y', 'left_x', 'left_y', 'right_x', 'right_y',
                   'radius', 'psie', 'cte']
list_cam_name = ['center_x', 'center_y', 'left_x', 'left_y', 'right_x', 'right_y']


class TimeoutError(Exception):
    pass


def timeout(seconds=10, error_message=os.strerror(errno.ETIME)):
    def decorator(func):
        def _handle_timeout(signum, frame):
            raise TimeoutError(error_message)

        def wrapper(*args, **kwargs):
            signal.signal(signal.SIGALRM, _handle_timeout)
            signal.setitimer(signal.ITIMER_REAL,seconds) #used timer instead of alarm
            try:
                result = func(*args, **kwargs)
            finally:
                signal.alarm(0)
            return result
        return wraps(func)(wrapper)
    return decorator

@timeout(1)
def recv_with_timeout(socket):
    return socket.recvfrom(1024)


def generate_msg_from_data(data_udp: payload_t, preview: dict):
    m = ACRaw()

    m.header.stamp = rospy.Time.now()

    # m.identifier = String(data=data_udp.identifier[0])
    m.size.data = data_udp.size
    m.speed_Kmh.data = data_udp.speed_Kmh
    m.speed_Mph.data = data_udp.speed_Mph
    m.speed_Ms.data = data_udp.speed_Ms
    m.isAbsEnabled.data = data_udp.isAbsEnabled
    m.isAbsInAction.data = data_udp.isAbsInAction
    m.isTcInAction.data = data_udp.isTcInAction
    m.isTcEnabled.data = data_udp.isTcEnabled
    m.isInPit.data = data_udp.isInPit
    m.isEngineLimiterOn.data = data_udp.isEngineLimiterOn
    m.accG_vertical.data = data_udp.accG_vertical
    m.accG_horizontal.data = data_udp.accG_horizontal
    m.accG_frontal.data = data_udp.accG_frontal
    m.lapTime.data = data_udp.lapTime
    m.lastLap.data = data_udp.lastLap
    m.bestLap.data = data_udp.bestLap
    m.lapCount.data = data_udp.lapCount
    m.gas.data = data_udp.gas
    m.brake.data = data_udp.brake
    m.clutch.data = data_udp.clutch
    m.engineRPM.data = data_udp.engineRPM
    m.steer.data = data_udp.steer
    m.gear.data = data_udp.gear
    m.cgHeight.data = data_udp.cgHeight

    # m.wheelAngularSpeed.layout.dim.append(MultiArrayDimension('val', 4, 4))
    # m.slipAngle.layout.dim.append(MultiArrayDimension('val', 4, 4))
    # m.slipAngle_ContactPatch.layout.dim.append(MultiArrayDimension('val', 4, 4))
    # m.slipRatio.layout.dim.append(MultiArrayDimension('val', 4, 4))
    # m.tyreSlip.layout.dim.append(MultiArrayDimension('val', 4, 4))
    # m.ndSlip.layout.dim.append(MultiArrayDimension('val', 4, 4))
    # m.load.layout.dim.append(MultiArrayDimension('val', 4, 4))
    # m.Dy.layout.dim.append(MultiArrayDimension('val', 4, 4))
    # m.Mz.layout.dim.append(MultiArrayDimension('val', 4, 4))
    # m.tyreDirtyLevel.layout.dim.append(MultiArrayDimension('val', 4, 4))
    # m.camberRAD.layout.dim.append(MultiArrayDimension('val', 4, 4))
    # m.tyreRadius.layout.dim.append(MultiArrayDimension('val', 4, 4))
    # m.tyreLoadedRadius.layout.dim.append(MultiArrayDimension('val', 4, 4))
    # m.suspensionHeight.layout.dim.append(MultiArrayDimension('val', 4, 4))

    for k in range(0, 4):
        m.wheelAngularSpeed.data.append(data_udp.wheelAngularSpeed[k])
        m.slipAngle.data.append(data_udp.slipAngle[k])
        m.slipAngle_ContactPatch.data.append(data_udp.slipAngle_ContactPatch[k])
        m.slipRatio.data.append(data_udp.slipRatio[k])
        m.tyreSlip.data.append(data_udp.tyreSlip[k])
        m.ndSlip.data.append(data_udp.ndSlip[k])
        m.load.data.append(data_udp.load[k])
        m.Dy.data.append(data_udp.Dy[k])
        m.Mz.data.append(data_udp.Mz[k])
        m.tyreDirtyLevel.data.append(data_udp.tyreDirtyLevel[k])
        m.camberRAD.data.append(data_udp.camberRAD[k])
        m.tyreRadius.data.append(data_udp.tyreRadius[k])
        m.tyreLoadedRadius.data.append(data_udp.tyreLoadedRadius[k])
        m.suspensionHeight.data.append(data_udp.suspensionHeight[k])
    m.carPositionNormalized.data = data_udp.carPositionNormalized
    m.carSlope.data = data_udp.carSlope

    # m.carCoordinates.layout.dim.append(MultiArrayDimension('val', 3, 3))

    for k in range(0, 3):
        m.carCoordinates.data.append(data_udp.carCoordinates[k])

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
        m.plane_center_x.data = np.array(preview['plane']['center_x'], dtype=np.float64)
        m.plane_center_y.data = np.array(preview['plane']['center_y'], dtype=np.float64)
        m.plane_left_x.data = np.array(preview['plane']['left_x'], dtype=np.float64)
        m.plane_left_y.data = np.array(preview['plane']['left_y'], dtype=np.float64)
        m.plane_right_x.data = np.array(preview['plane']['right_x'], dtype=np.float64)
        m.plane_right_y.data = np.array(preview['plane']['right_y'], dtype=np.float64)
        m.plane_radius.data = np.array(preview['plane']['radius'], dtype=np.float64)
        m.plane_psie.data = preview['plane']['psie']
        m.plane_cte.data = preview['plane']['cte']
        m.plane_speed.data = np.array(preview['plane']['speed'])

        # cam = [center_x, center_y, left_x, left_y, right_x, right_y]
        m.cam_center_x.data = np.array(preview['cam']['center_x'], dtype=np.float64)
        m.cam_center_y.data = np.array(preview['cam']['center_y'], dtype=np.float64)
        m.cam_left_x.data = np.array(preview['cam']['left_x'], dtype=np.float64)
        m.cam_left_y.data = np.array(preview['cam']['left_y'], dtype=np.float64)
        m.cam_right_x.data = np.array(preview['cam']['right_x'], dtype=np.float64)
        m.cam_right_y.data = np.array(preview['cam']['right_y'], dtype=np.float64)

    return m


if __name__ == '__main__':
    # Parameters
    name_node = 'ac_pub'
    freq_pub = rospy.get_param('/{}/freq_pub'.format(name_node), 500)
    print('freq_pub : {}'.format(freq_pub))

    rospy.init_node(name_node)

    host = rospy.get_param('/{}/host'.format(name_node), 'localhost')
    port = rospy.get_param('/{}/port'.format(name_node), 9996)

    course = rospy.get_param('/{}/course'.format(name_node), 'imola')
    print('Target course : {}'.format(course))

    aipath = ai('/home/rnd/.steam/steam/steamapps/common/assettocorsa', course, debug_logger)

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
            # data, _ = s.recvfrom(1024)
            data, _ = recv_with_timeout(s)
            
            if len(data) != 328:
                print('Inconsistent data length : {}'.format(len(data)))
                continue
            payload = payload_t.from_buffer_copy(data)

            # Get additional data
            aipath.recv_telemetry(payload)

            # Convert data to msg
            m = generate_msg_from_data(payload, aipath.preview)

            pub.publish(m)

            rate.sleep()
        except TimeoutError:
            dismiss(s, target)
            time.sleep(0.5)
            s = socket(AF_INET, SOCK_DGRAM)
            s.bind((ip_bind, 0))
            while True:
                if rospy.is_shutdown():
                    break
                if not target_on_listening(s, target):
                    print('Waiting again...')
                    time.sleep(0.5)
                else:
                    break
            handshake(s, target)
            print('Connected again')
            continue
        except KeyboardInterrupt:
            with suppress(TimeoutError):
                print('Finishing...')
                dismiss(s, target)
                break

    dismiss(s, target)
    print('{} node finished.'.format(name_node))
