#!/usr/bin/python3
"""
    Assetto Corsa Velocity Prediction

    Jinsun Park
    (zzangjinsun@3secondz.com)
"""


import rospy
from assetto_corsa.msg import ACRaw
from helper import DataHelper


def callback(data: ACRaw):
    global rate, freq_sub, t_accumulate, preview_dist, hist_speed

    px = list(data.plane_center_x.data)
    py = list(data.plane_center_y.data)
    speed = data.speed_Kmh.data

    dh = DataHelper(localX=px, localY=py, speed=speed)
    dh.set_preview_distance(preview_dist)
    preview = dh.get_preview(0, 'DISTANCE')

    curvature = preview['Curvature']

    ##### PREDICTION CODE HERE #####
    # input : curvature, speed, hist_speed

    # output : predicted speed
    output = [1, 2, 3, 4, 5]
    ##### PREDICTION CODE HERE #####

    rospy.loginfo(output)

    # Speed history
    hist_speed.append(speed)
    num_hist = len(hist_speed)
    if num_hist > t_accumulate*freq_sub:
        hist_speed = hist_speed[num_hist - t_accumulate*freq_sub:]

    rate.sleep()


if __name__ == '__main__':
    global rate, freq_sub, t_accumulate, preview_dist, hist_speed

    # Parameters
    name_node = 'ac_predict'
    freq_sub = rospy.get_param('~freq_sub', 20)
    t_accumulate = rospy.get_param('~t_accumulate', 2)
    preview_dist = rospy.get_param('~preview_dist', 100)
    topic_name = rospy.get_param('~topic_name', '/ac_pub/ACRaw')

    hist_speed = []

    rospy.loginfo('Freq : {}'.format(freq_sub))
    rospy.loginfo('t_accumulate : {}'.format(t_accumulate))

    rospy.init_node(name_node)

    rate = rospy.Rate(freq_sub)

    sub = rospy.Subscriber(topic_name, ACRaw, callback)

    rospy.spin()
