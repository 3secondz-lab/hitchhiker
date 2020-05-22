#!/usr/bin/env python


from moviepy.editor import VideoFileClip
#import classLine
import datetime as dt
import time
import numpy as np
import cv2

from adjustThreshold import adjust_threshold
from thresholdColorGrad import combined_sobels, rgb2lab, rgb2hls, hls_wy_bin, threshold, hls_select, lab_select, luv_select, abs_sobel_th, mag_sobel_th, dir_sobel_th
from findLane import mask_roi, mask_window, find_window_centroids, show_window_centroids, polyfit_window, measure_curve_r, get_offset

#file_path = "../../ac_laguna_mx5_2.mp4"
file_path = "../../ac_inje_86_2.mp4"
cap = cv2.VideoCapture(file_path)
#threshold = [88, 255]#hls
threshold = [155, 255]#lab
kernel = 9
direction = [0.7, 1.3]
#mode = 'adjust'
mode = 'fixed'
#crop = [484,812]#laguna seca
crop = [240,600]#inje

if (cap.isOpened() == False):
    print("Error opening video stream or file")

while(cap.isOpened()):
    ret, img = cap.read()
    if ret == True:

        if (mode == 'adjust'):
            key = cv2.waitKey(10)
            threshold, direction, kernel = adjust_threshold(key, threshold, direction, kernel)

            threshold_hls = threshold
            #direction_hls = direction
            #kernel_hls = kernel

            threshold_lab = threshold
            #direction_hls = direction
            #kernel_hls = kernel

            threshold_abx = threshold
            #direction_abx = direction
            kernel_abx = kernel

            threshold_aby = threshold
            #direction_aby = direction
            kernel_aby = kernel

            threshold_mag = threshold
            #direction_mag = direction
            kernel_mag = kernel

            #threshold_dir = threshold
            direction_dir = direction
            kernel_dir = kernel

        else:
            threshold_hls = (88,255)
            #direction_hls = direction
            #kernel_hls = kernel

            threshold_lab = (155,255)
            #direction_hls = direction
            #kernel_hls = kernel

            threshold_abx = (20,120)
            #direction_abx = direction
            kernel_abx = 15

            threshold_aby = (20,120)
            #direction_aby = direction
            kernel_aby = 15

            threshold_mag = (80,200)
            #direction_mag = direction
            kernel_mag = 15

            #threshold_dir = threshold
            direction_dir = (np.pi/4, np.pi/2)
            kernel_dir = 15


        #if key == 27: # ESC
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break


        #img_roi = mask_roi(img)
        img_roi = img[crop[0]:crop[1],:]
        img_hls = hls_select(img_roi, ch='s', thresh=threshold_hls)
        img_lab = lab_select(img_roi, ch='l', thresh=threshold_lab)*255
        #img_lab = rgb2lab(img_roi)[:,:,0]
        img_grad_abx = abs_sobel_th(img_lab, orient='x', ksize=kernel_abx, thresh=threshold_abx)
        img_grad_aby = abs_sobel_th(img_lab, orient='y', ksize=kernel_aby, thresh=threshold_aby)
        img_grad_mag = mag_sobel_th(img_lab, ksize=kernel_mag, thresh=threshold_mag)
        img_combined = combined_sobels(img_grad_abx, img_grad_aby, img_grad_mag, img_lab, kernel_size=15, angle_thres=(np.pi/4, np.pi/2))
        #img_wy = hls_wy_bin(img_roi)
        img_bin = np.zeros_like(img_combined)
        img_bin[(img_combined == 1) | (img_hls == 1)] = 1


        copy_combined = np.copy(img_bin)
        #print(copy_combined.shape[0], copy_combined.shape[1])#360, 1280
        #(bottom_px, right_px) = (copy_combined.shape[0] - 1, copy_combined.shape[1] - 1) 
        (bottom_px, right_px) = (360 - 1, 1280 - 1) 
        pts = np.array([[210,bottom_px],[595,240],[690,240], [1110, bottom_px]], np.int32)
        cv2.polylines(copy_combined,[pts],True,(255,255,255), 10)

        img_fin = copy_combined
        cv2.imshow("test", img_fin*255)



    else:
        break

cap.release()
cv2.destroyAllWindows()