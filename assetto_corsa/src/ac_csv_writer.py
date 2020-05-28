#!/usr/bin/python3
"""
    Assetto Corsa CSV writer

    Jinsun Park
    (zzangjinsun@3secondz.com)
"""


import rospy
from assetto_corsa.msg import ACRaw
import csv


def callback(data: ACRaw):
    global rate, csv_writer

    t = data.header.stamp.to_sec()
    px = data.carCoordinates.data[0]
    py = data.carCoordinates.data[2]
    height = data.carCoordinates.data[1]

    ax = data.accG_frontal.data
    ay = data.accG_horizontal.data
    az = data.accG_vertical.data

    gps_speed = data.speed_Kmh.data
    acc = data.gas.data
    brk = data.brake.data
    steer = data.steer.data
    pos_norm = data.carPositionNormalized.data

    output = [t, px, py, height, ax, ay, az, gps_speed, acc, brk, steer, pos_norm]

    csv_writer.writerow(output)

    print(output)

    rate.sleep()


if __name__ == '__main__':
    # Parameters
    node_name = 'ac_csv_writer'

    rospy.init_node(node_name, anonymous=False)

    global rate, path_out, file_csv, csv_writer

    topic_name = rospy.get_param('/{}/topic_name'.format(node_name), '/ac_pub/ACRaw')
    freq_sub = rospy.get_param('/{}/freq_sub'.format(node_name), 10)
    path_out = rospy.get_param('/{}/path_out'.format(node_name), 'output.csv')

    print('Topic name : {}'.format(topic_name))
    print('Freq sub : {}'.format(freq_sub))
    print('Path out : {}'.format(path_out))

    rate = rospy.Rate(freq_sub)

    sub = rospy.Subscriber(topic_name, ACRaw, callback, queue_size=1)

    try:
        file_csv = open(path_out, 'w', encoding='utf-8')
        csv_writer = csv.writer(file_csv)
        csv_writer.writerow(["TimeStamp", "PosLocalX", "PosLocalY", "Height",
                             "AccelX", "AccelY", "AccelZ",
                             "GPS_Speed", "PedalPosAcc", "PedalPosBrk", "AngleSteer",
                             "CarPositionNormalized"])
    except OSError:
        print('Failed to open : {}'.format(path_out))

    rospy.spin()
