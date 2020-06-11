import rospy
from assetto_corsa.msg import ACRaw
from helper import DataHelper

from chpt_d1_3sec.test_model import Model

# dataName = 'IJF_D1'.format(driverNum)  # cWindow [s], vWindow [s], vpWindow [s], cUnit [Hz], vUnit [Hz], vpUnit [Hz]
dataName = 'IJF_D1'  # cWindow [s], vWindow [s], vpWindow [s], cUnit [Hz], vUnit [Hz], vpUnit [Hz]
chpt_encC_path = './BEST_checkpoint_ENCC_{}.pth.tar'.format(dataName)
chpt_encD_path = './BEST_checkpoint_ENCD_{}.pth.tar'.format(dataName)
chpt_dec_path = './BEST_checkpoint_DEC_{}.pth.tar'.format(dataName)
chpt_stat_path = './BEST_stat_{}.pickle'.format(dataName)
testModel = Model(chpt_encC_path, chpt_encD_path, chpt_dec_path, chpt_stat_path)  # speed prediction에 사용할 model parameters

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
    # output = [1, 2, 3, 4, 5]

    preds, alphas_d, alphas_c = testModel.predict(curvature, speed, hist_speed)
    # input : curvature (np.ndarray, shape: (len_c,)),
    #         speed (np.float),
    #         hist_speed (np.ndarray, shape: (len_d,), 0.05초 간격으로 과거 2초간 속력을 사용했었음)
    # output : preds (torch.Tensor, shape: (1, 20), 0.1초 간격으로 2초앞까지의 예측 속력, [0][0]이 0.1초 후 예측 속력),
    #          alphas_d (torch.Tensor, shape: (1, 20, len_d), 매 예측시 사용된 attention weight 값),
    #          alphas_c (torch.Tensor, shape: (1, 20, len_c), 매 예측시 사용된 attention weight 값)

    ##### PREDICTION CODE HERE #####

    print(preds)
    print(alphas_d)
    print(alphas_c)

    # rospy.loginfo(preds)
    # rospy.loginfo(alphas_d)
    # rospy.loginfo(alphas_c)

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
