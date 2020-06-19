from socket import *
import struct
import argparse
from ctypes import *
import ctypes
import os
import time


class payload_t(Structure):
    _fields_ = [
        ('identifier', c_wchar),
        ('size', c_int),
        ('speed_Kmh', c_float),
        ('speed_Mph', c_float),
        ('speed_Ms', c_float),
        ('isAbsEnabled', c_bool),
        ('isAbsInAction', c_bool),
        ('isTcInAction', c_bool),
        ('isTcEnabled', c_bool),
        ('isInPit', c_bool),
        ('isEngineLimiterOn', c_bool),
        ('accG_vertical', c_float),
        ('accG_horizontal', c_float),
        ('accG_frontal', c_float),
        ('lapTime', c_int),
        ('lastLap', c_int),
        ('bestLap', c_int),
        ('lapCount', c_int),
        ('gas', c_float),
        ('brake', c_float),
        ('clutch', c_float),
        ('engineRPM', c_float),
        ('steer', c_float),
        ('gear', c_int),
        ('cgHeight', c_float),
        ('wheelAngularSpeed', c_float * 4),
        ('slipAngle', c_float * 4),
        ('slipAngle_ContactPatch', c_float * 4),
        ('slipRatio', c_float * 4),
        ('tyreSlip', c_float * 4),
        ('ndSlip', c_float * 4),
        ('load', c_float * 4),
        ('Dy', c_float * 4),
        ('Mz', c_float * 4),
        ('tyreDirtyLevel', c_float * 4),
        ('camberRAD', c_float * 4),
        ('tyreRadius', c_float * 4),
        ('tyreLoadedRadius', c_float * 4),
        ('suspensionHeight', c_float * 4),
        ('carPositionNormalized', c_float),
        ('carSlope', c_float),
        ('carCoordinates', c_float * 3),
    ]


handshaker = 'iii'
port = 9996

identifier = None
version = None


def target_on_listening(s: socket, target: tuple):
    host = target[0] if target[0] != 'localhost' else '127.0.0.1'
    port = target[1]

    s.settimeout(1)

    try:
        SYN = struct.pack('iii', 1, 1, 0)
        s.sendto(SYN, target)
        s.recvfrom(1024)
        s.settimeout(None)
        return True
    except timeout:
        s.settimeout(None)
        return False

    # res = os.system(f'nc -vnzu {host} {port} > /dev/null 2>&1')
    # if res == 0:
    #     return True
    # else:
    #     return False


def decode_synack(synack):
    car_name = synack[0:100]
    driver_name = synack[100:200]
    identifier = struct.unpack('i', synack[200:204])[0]
    version = struct.unpack('i', synack[204:208])[0]
    track_name = synack[208:308]
    track_config = synack[308:408]

    return car_name, driver_name, identifier, version, track_name, track_config


def handshake(s: socket, target: tuple):
    global identifier, version
    SYN = struct.pack('iii', 1, 1, 0)
    print('Handshake : Send SYN')
    s.sendto(SYN, target)
    print('Handshake : Receive SYNACK')
    SYNACK, addr = s.recvfrom(1024)
    _, _, identifier, version, _, _ = decode_synack(SYNACK)
    ACK = struct.pack('iii', identifier, version, 1)
    print('Handshake : Send ACK')
    s.sendto(ACK, target)
    print('Handshake : Done')

def update(s: socket, target: tuple):
    global identifier, version
    SYN = struct.pack('iii', identifier, version, 2)
    s.sendto(SYN, target)


def dismiss(s: socket, target: tuple):
    FIN = struct.pack('iii', identifier, version, 3)
    print('Send FIN packet to target')
    s.sendto(FIN, target)


def decode(s):
    while True:
        try:
            data, _ = s.recvfrom(1024)
            if len(data) != 328:
                print(len(data))
            payload = payload_t.from_buffer_copy(data)
            # print(sizeof(payload))
            print(payload.carPositionNormalized, payload.carSlope)
        except KeyboardInterrupt:
            dismiss(s, target)
            break


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='localhost')
    args = parser.parse_args()

    target = (args.host, port)
    # print(sizeof(payload_t))
    print(f'Target: {args.host}:{port}')

    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(('localhost', 0))
    print(f'Local socket: localhost:{s.getsockname()[1]}')

    print('Waiting for 9996 listening...')
    while not target_on_listening(target):
        time.sleep(0.5)
    print('Target on listening.')

    handshake(s, target)
    decode(s)
