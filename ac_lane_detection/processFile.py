#!/usr/bin/env python


from moviepy.editor import VideoFileClip
#import classLine
import datetime as dt
import time
import numpy as np
import cv2

from adjustThreshold import adjust_threshold
from thresholdColorGrad import combined_color, line_wr_bin, combined_sobels, rgb2gray, rgb2luv, rgb2lab, rgb2hls, hls_wy_bin, threshold, hls_select, lab_select, luv_select, abs_sobel_th, mag_sobel_th, dir_sobel_th
from findLane import mask_roi, mask_window, find_window_centroids, show_window_centroids, polyfit_window, measure_curve_r, get_offset

#file_path = "../../ac_laguna_mx5_2.mp4"
file_path = "../../ac_inje_86_2.mp4"
cap = cv2.VideoCapture(file_path)
thresh = [64, 255]#hls
#thresh = [94, 255]#lab
#thresh = [94, 255]#luv
kernel = 11
direction = [0.7, 1.3]
mode = 'adjust'
#mode = 'fixed'
#crop = [484,812]#laguna seca
crop = [240,600]#inje

if (cap.isOpened() == False):
    print("Error opening video stream or file")

while(cap.isOpened()):
    ret, img = cap.read()
    if ret == True:

        if (mode == 'adjust'):
            key = cv2.waitKey(10)
            thresh, direction, kernel = adjust_threshold(key, thresh, direction, kernel)

            thresh_hls = thresh
            #direction_hls = direction
            #kernel_hls = kernel

            thresh_lab = thresh
            #direction_lab = direction
            #kernel_lab = kernel

            thresh_luv = thresh
            #direction_luv = direction
            #kernel_luv = kernel

            thresh_abx = thresh
            #direction_abx = direction
            kernel_abx = kernel

            thresh_aby = thresh
            #direction_aby = direction
            kernel_aby = kernel

            thresh_mag = thresh
            #direction_mag = direction
            kernel_mag = kernel

            #thresh_dir = thresh
            direction_dir = direction
            kernel_dir = kernel

        else:
            thresh_hls = (64,255)
            #direction_hls = direction
            #kernel_hls = kernel

            thresh_lab = (94,255)
            #direction_lab = direction
            #kernel_lab = kernel

            thresh_luv = (92,255)
            #direction_luv = direction
            #kernel_luv = kernel

            thresh_abx = (20,120)
            #direction_abx = direction
            kernel_abx = 11

            thresh_aby = (20,120)
            #direction_aby = direction
            kernel_aby = 11

            thresh_mag = (80,200)
            #direction_mag = direction
            kernel_mag = 11

            #thresh_dir = thresh
            direction_dir = (np.pi/4, np.pi/2)
            kernel_dir = 11


        #if key == 27: # ESC
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break


        #img_roi = mask_roi(img)
        img_roi = img[crop[0]:crop[1],:]
        img_resized = cv2.resize(img_roi, (640,180), interpolation = cv2.INTER_AREA)
        img_blurred = cv2.GaussianBlur(img_resized,(kernel,kernel),25)
        #img_blurred = cv2.GaussianBlur(img_resized,(15,15),kernel)
        #img_hls = hls_select(img_blurred, ch='s', thresh=thresh_hls)*255
        img_hls = rgb2hls(img_blurred)[:,:,2]
        #img_lab = lab_select(img_blurred, ch='l', thresh=thresh_lab)*255
        img_lab = rgb2lab(img_blurred)[:,:,0]
        #img_lab = img_hls
        img_color = combined_color(img_blurred)
        img_grad_abx = abs_sobel_th(img_hls, orient='x', ksize=kernel_abx, thresh=thresh_abx)
        img_grad_aby = abs_sobel_th(img_hls, orient='y', ksize=kernel_aby, thresh=thresh_aby)
        img_grad_mag = mag_sobel_th(img_hls, ksize=kernel_mag, thresh=thresh_mag)
        img_combined = combined_sobels(img_grad_abx, img_grad_aby, img_grad_mag, img_hls, kernel_size=15, angle_thres=(np.pi/4, np.pi/2))
        img_wr = line_wr_bin(img_blurred)
        img_bin = np.zeros_like(img_combined)
        #img_bin[(img_combined == 1) | (img_hls == 1)] = 1
        img_bin[(img_combined == 1) | (img_wr == 1)] = 1


        copy_combined = np.copy(img_bin)
        #print(copy_combined.shape[0], copy_combined.shape[1])#180*2, 640*2
        #(bottom_px, right_px) = (copy_combined.shape[0] - 1, copy_combined.shape[1] - 1) 
        (bottom_px, right_px) = (180*2 - 1, 640*2 - 1) 
        pts = np.array([[210,bottom_px],[595,240],[690,240], [1110, bottom_px]], np.int32)
        cv2.polylines(copy_combined,[pts],True,(255,255,255), 10)

        #img_fin = img_combined
        img_fin = img_bin
        #img_fin = img_roi
        #img_fin = cv2.vconcat(cv2.vconcat(cv2.vconcat(img_roi,img_hls), img_lab),img_combined)
        #img_fin = cv2.vconcat(img_roi,img_hls)
        #img_fin=img_resized

        #cv2.imshow("fin", img_fin*255)

        

        '''
        #img_test = np.zeros((180*2, 640*2, 3), np.uint8)
        img_test = np.zeros_like(img_roi)
        img_test[180*0:180*1,640*0:640*1,0:3]=img_resized
        img_test[180*0:180*1,640*1:640*2,0:3]=img_blurred
        
        img_test[180*1:180*2,640*0:640*1,0]=img_hls
        img_test[180*1:180*2,640*0:640*1,1]=img_hls
        img_test[180*1:180*2,640*0:640*1,2]=img_hls
        img_test[180*1:180*2,640*1:640*2,0]=img_lab
        img_test[180*1:180*2,640*1:640*2,1]=img_lab
        img_test[180*1:180*2,640*1:640*2,2]=img_lab
        '''

        img_test = np.zeros((180*4,640*3,3),np.uint8)
        img_test[180*0:180*1,640*0:640*1,:]=img_resized
        img_test[180*0:180*1,640*1:640*2,:]=img_blurred
        img_test[180*0:180*1,640*2:640*3,:] = cv2.cvtColor(combined_color(img_blurred)*255, cv2.COLOR_GRAY2RGB)
        
        '''
        #HLS, Lab, LUV analysis
        img_test[180*1:180*2,640*0:640*1,:]=cv2.cvtColor(threshold(rgb2hls(img_blurred)[:,:,0],thresh_hls)*255,cv2.COLOR_GRAY2RGB)
        img_test[180*2:180*3,640*0:640*1,:]=cv2.cvtColor(threshold(rgb2hls(img_blurred)[:,:,1],thresh_hls)*255,cv2.COLOR_GRAY2RGB)
        img_test[180*3:180*4,640*0:640*1,:]=cv2.cvtColor(threshold(rgb2hls(img_blurred)[:,:,2],thresh_hls)*255,cv2.COLOR_GRAY2RGB)

        img_test[180*1:180*2,640*1:640*2,:]=cv2.cvtColor(threshold(rgb2lab(img_blurred)[:,:,0],thresh_lab)*255,cv2.COLOR_GRAY2RGB)
        img_test[180*2:180*3,640*1:640*2,:]=cv2.cvtColor(threshold(rgb2lab(img_blurred)[:,:,1],thresh_lab)*255,cv2.COLOR_GRAY2RGB)
        img_test[180*3:180*4,640*1:640*2,:]=cv2.cvtColor(threshold(rgb2lab(img_blurred)[:,:,2],thresh_lab)*255,cv2.COLOR_GRAY2RGB)

        img_test[180*1:180*2,640*2:640*3,:]=cv2.cvtColor(threshold(rgb2luv(img_blurred)[:,:,0],thresh_luv)*255,cv2.COLOR_GRAY2RGB)
        img_test[180*2:180*3,640*2:640*3,:]=cv2.cvtColor(threshold(rgb2luv(img_blurred)[:,:,1],thresh_luv)*255,cv2.COLOR_GRAY2RGB)
        img_test[180*3:180*4,640*2:640*3,:]=cv2.cvtColor(threshold(rgb2luv(img_blurred)[:,:,2],thresh_luv)*255,cv2.COLOR_GRAY2RGB)
        
        
        #Red&White corner line analysis
        #img_test[180*0:180*1,640*2:640*3,:] = cv2.cvtColor(line_wr_bin(img_blurred)*255, cv2.COLOR_GRAY2RGB)
        #img_test[180*1:180*2,640*0:640*1,:]=cv2.cvtColor(threshold(rgb2hls(img_blurred)[:,:,2],thresh_hls)*255,cv2.COLOR_GRAY2RGB)
        #img_test[180*1:180*2,640*1:640*2,:]=cv2.cvtColor((1-threshold(rgb2lab(img_blurred)[:,:,2],thresh_lab))*255,cv2.COLOR_GRAY2RGB)
        #img_test[180*1:180*2,640*2:640*3,:]=cv2.cvtColor((threshold(rgb2luv(img_blurred)[:,:,1],thresh_luv))*q255,cv2.COLOR_GRAY2RGB)
        #img_test[180*2:180*3,640*0:640*1,:]=cv2.cvtColor(threshold(rgb2gray(img_blurred),thresh)*255,cv2.COLOR_GRAY2RGB)
        #img_test[180*2:180*3,640*1:640*2,:]=cv2.cvtColor(threshold(img_blurred[:,:,0],thresh)*255,cv2.COLOR_GRAY2RGB)
        #img_test[180*2:180*3,640*2:640*3,:]=cv2.cvtColor((1-threshold(rgb2luv(img_blurred)[:,:,1],thresh_luv))*255,cv2.COLOR_GRAY2RGB)
        '''


        
        #Gradient analysis
        img_test[180*1:180*2,640*0:640*1,:] = cv2.cvtColor(abs_sobel_th(img_color, orient='x', ksize=kernel_abx, thresh=thresh_abx)*255, cv2.COLOR_GRAY2RGB)
        img_test[180*1:180*2,640*1:640*2,:] = cv2.cvtColor(abs_sobel_th(img_color, orient='y', ksize=kernel_abx, thresh=thresh_abx)*255, cv2.COLOR_GRAY2RGB)
        img_test[180*1:180*2,640*2:640*3,:] = cv2.cvtColor(mag_sobel_th(img_color, ksize=kernel_abx, thresh=thresh_abx)*255, cv2.COLOR_GRAY2RGB)

        img_test[180*2:180*3,640*0:640*1,:] = cv2.cvtColor(abs_sobel_th(img_hls, orient='x', ksize=kernel_abx, thresh=thresh_abx)*255, cv2.COLOR_GRAY2RGB)
        img_test[180*2:180*3,640*1:640*2,:] = cv2.cvtColor(abs_sobel_th(img_hls, orient='y', ksize=kernel_abx, thresh=thresh_abx)*255, cv2.COLOR_GRAY2RGB)
        img_test[180*2:180*3,640*2:640*3,:] = cv2.cvtColor(mag_sobel_th(img_hls, ksize=kernel_abx, thresh=thresh_abx)*255, cv2.COLOR_GRAY2RGB)

        





        cv2.imshow("test", img_test)

        ##Note##
        #kernel = 11, thresh = 64 good for S in HLS --> Grass/road separation
        #kernel = 11, thresh = 94 good for b in Lab --> RedWhite separation
        #kernel = 11, thresh = 92 good for U in LUV --> RedWhite separation
        #kernel = 11, thresh = 190 good for W in GRAY --> White separation



        
        


    else:
        break

cap.release()
cv2.destroyAllWindows()