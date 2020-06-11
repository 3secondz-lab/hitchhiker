#!/usr/bin/python3
"""
    Assetto Corsa Joystick Controller

    Jinsun Park
    (zzangjinsun@3secondz.com)
"""

import rospy
import numpy as np
import os
import pickle

from assetto_corsa.msg import ACRaw
from helper import DataHelper

from chpt_d1_3sec.test_model import Model

# dataName = 'IJF_D1'.format(driverNum)  # cWindow [s], vWindow [s], vpWindow [s], cUnit [Hz], vUnit [Hz], vpUnit [Hz]
dataName = 'IJF_D1'  # cWindow [s], vWindow [s], vpWindow [s], cUnit [Hz], vUnit [Hz], vpUnit [Hz]
chpt_encC_path = 'chpt_d1_3sec/BEST_checkpoint_ENCC_{}.pth.tar'.format(dataName)
chpt_encD_path = 'chpt_d1_3sec/BEST_checkpoint_ENCD_{}.pth.tar'.format(dataName)
chpt_dec_path = 'chpt_d1_3sec/BEST_checkpoint_DEC_{}.pth.tar'.format(dataName)
chpt_stat_path = 'chpt_d1_3sec/BEST_stat_{}.pickle'.format(dataName)
print('Load model...')
testModel = Model(chpt_encC_path, chpt_encD_path, chpt_dec_path, chpt_stat_path)  # speed prediction에 사용할 model parameters
print('Load model finished.')


def callback(data: ACRaw):
    global rate, freq_sub, t_accumulate, preview_dist, hist_speed, flag_debug, path_debug

    px = list(data.plane_center_x.data)
    py = list(data.plane_center_y.data)
    speed = np.float(data.speed_Kmh.data)

    dh = DataHelper(localX=px, localY=py, speed=speed)
    dh.set_preview_distance(preview_dist)
    preview = dh.get_preview(0, 'DISTANCE')

    curvature = preview['Curvature']

    # Speed history
    hist_speed.append(speed)
    num_hist = len(hist_speed)
    if num_hist > t_accumulate*freq_sub:
        hist_speed = hist_speed[num_hist - t_accumulate*freq_sub:]

    hist_speed_ndarray = np.array(hist_speed)

    ##### PREDICTION CODE HERE #####
    # input : curvature (np.ndarray, shape: (len_c,)),
    #         speed (np.float),
    #         hist_speed (np.ndarray, shape: (len_d,), 0.05초 간격으로 과거 2초간 속력을 사용했었음)
    # output : preds (torch.Tensor, shape: (1, 20), 0.1초 간격으로 2초앞까지의 예측 속력, [0][0]이 0.1초 후 예측 속력),
    #          alphas_d (torch.Tensor, shape: (1, 20, len_d), 매 예측시 사용된 attention weight 값),
    #          alphas_c (torch.Tensor, shape: (1, 20, len_c), 매 예측시 사용된 attention weight 값)

    if curvature.size == 0:
        rospy.loginfo('Curvature is empty.')
        return

    if not hist_speed:
        rospy.loginfo('hist_speed is empty.')
        return

    preds, alphas_d, alphas_c = testModel.predict(curvature, speed, hist_speed_ndarray)

    preds = preds.data.cpu().numpy()
    alphas_d = alphas_d.data.cpu().numpy()
    alphas_c = alphas_c.data.cpu().numpy()

    ##### PREDICTION CODE HERE #####

    if flag_debug:
        t = data.header.stamp.to_nsec()
        path_out = '{}/{:019d}.bin'.format(path_debug, t)

        try:
            debug_data = {
                'curvature': curvature,
                'speed': speed,
                'hist_speed': hist_speed_ndarray,
                'preds': preds,
                'alphas_d': alphas_d,
                'alphas_c': alphas_c
            }

            f = open(path_out, 'wb')
            pickle.dump(debug_data, f)
            f.close()

            # NOTE : For debug data loading,
            # filename = '12345.bin'
            # with open(filename, 'rb') as f:
            #   data = pickle.load(f)

            rospy.loginfo('Saved to {}'.format(path_out))
        except Exception as e:
            print(e)

    rate.sleep()


if __name__ == '__main__':
    global rate, freq_sub, t_accumulate, preview_dist, hist_speed, flag_debug, path_debug

    # Parameters
    name_node = 'ac_predict'

    rospy.init_node(name_node)

    freq_sub = rospy.get_param('~freq_sub', 20)
    t_accumulate = rospy.get_param('~t_accumulate', 2)
    preview_dist = rospy.get_param('~preview_dist', 100)
    topic_name = rospy.get_param('~topic_name', '/ac_pub/ACRaw')

    flag_debug = rospy.get_param('~debug', False)
    path_debug = rospy.get_param('~path_debug', './')

    hist_speed = []

    rospy.loginfo('Freq : {}'.format(freq_sub))
    rospy.loginfo('t_accumulate : {}'.format(t_accumulate))

    if flag_debug:
        rospy.loginfo('debug mode is on.')
        rospy.loginfo('path_debug : {}'.format(path_debug))
        os.makedirs(path_debug, exist_ok=True)

    rate = rospy.Rate(freq_sub)

    sub = rospy.Subscriber(topic_name, ACRaw, callback, queue_size=1)

    rospy.spin()
