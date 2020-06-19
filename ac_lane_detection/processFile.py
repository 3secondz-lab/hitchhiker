#!/usr/bin/env python


from moviepy.editor import VideoFileClip
#import classLine
import datetime as dt
import time
import numpy as np
import cv2

import matplotlib.pyplot as plt

from adjustThreshold import adjust_threshold
from thresholdColorGrad import combined_color, line_rgw_bin, line_wr_bin, combined_sobels, rgb2gray, rgb2luv, rgb2lab, rgb2hls, hls_wy_bin, threshold, hls_select, lab_select, luv_select, abs_sobel_th, mag_sobel_th, dir_sobel_th
from findLane import draw_hist, mask_roi, mask_window, find_window_centroids, show_window_centroids, polyfit_window, measure_curve_r, get_offset
from transformImage import undistort, corners_unwarp

from classLaneLine import LaneLine
from classLaneLineHistory import LaneLineHistory
from classAdvLaneDetectMem import AdvLaneDetectMem

#file_path = "../../ac_laguna_mx5_2.mp4"
#file_path = "../../ac_inje_86_2.mp4"
file_path = "../../ac_inje_lancer_1_2.mp4"
cap = cv2.VideoCapture(file_path)
thresh = [64, 255]#hls
kernel = 7
direction = [0.7, 1.3]
#mode = 'adjust'
mode = 'fixed'
#crop = [484,812,0,1280]#laguna seca - top, bottom, left, right
#crop = [240,600,0,1280]#inje_86 - top, bottom, left, right
#[h_img, w_img] = [360, 1280]#inje_86
crop = [60,310,220,1060]#inje_lancer - top, bottom, left, right
[h_cropped, w_cropped] = [crop[1]-crop[0], crop[3]-crop[2]]#inje_lancer
resize_rate = 0.5
[h_resized, w_resized] = [int(h_cropped*resize_rate), int(w_cropped*resize_rate)]
[h_src, w_src] = [60,130]
[h_dst, w_dst] = [20,60]
[h_win, w_win] = [10,30]


font = cv2.FONT_HERSHEY_DUPLEX

if (cap.isOpened() == False):
    print("Error opening video stream or file")

if (mode == 'adjust'):
    guide1 = "[Adjust mode]"
    guide2 = "TY:-/+ lower threshold(0~upper), UI: -/+ upper threshold(lower~255)"
    guide3 = "GH:-/+ lower direction(-pi~upper), JK:-/+ upper direction(lower~pi)"
    guide4 = "BN:-/+ kernel size"
else:
    guide1 = "[Adjust mode]"
    guide2 = "TY:-/+ lower threshold(0~upper), UI: -/+ upper threshold(lower~255)"
    guide3 = "GH:-/+ lower direction(-pi~upper), JK:-/+ upper direction(lower~pi)"
    guide4 = "BN:-/+ kernel size"




while(cap.isOpened()):
    ret, img = cap.read()
    if ret == True:
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        #cv2.imshow("test", img)
        img_cropped = img[crop[0]:crop[1],crop[2]:crop[3]]
        img_resized = cv2.resize(img_cropped, (w_resized,h_resized), interpolation = cv2.INTER_AREA)

        (bottom_px, right_px) = (img_resized.shape[0] - 1, img_resized.shape[1] - 1) 
        pts = np.array([[0,bottom_px],[w_src,h_src],[w_resized-w_src,h_src], [w_resized, bottom_px]], np.int32)#LB, LT, RT, RB, [x,y]
        src_pts = pts.astype(np.float32)
        dst_pts = np.array([[w_dst,bottom_px-h_dst], [w_dst,h_dst], [w_resized-w_dst,20], [w_resized-w_dst, bottom_px-h_dst]], np.float32)
        
        lane_detection = AdvLaneDetectMem(src_pts, dst_pts, 20, 100, 50)

        img_proc = lane_detection.process_image(img_resized)

        cv2.imshow("Result", img_proc)





        











    else:
        break

cap.release()
cv2.destroyAllWindows()