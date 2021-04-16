#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import cv2
import numpy as np
import math

#图像检测单元
#检测视野中的车道线， 停止线
#返回车相对车道线中心的误差，车道线的曲率，以及可视化车道线
class Keeper:
    def __init__(self):

        pass


    def stop_line(self,img):
       
        self.height = img.shape[0]
        self.width = img.shape[1]

        self.binary = self.preprocessing(img)
        #self.binary = self.binary[200:self.height-1,200:620]

        edges = cv2.Canny(img, 80, 120)
        edges[:150,:] = 0
        lines = cv2.HoughLinesP(edges, 1, math.pi/2, 5, None, 20, 1)
       
        if lines is not None:
            for line in lines[0]:  
                pt1 = (line[0],line[1])
                pt2 = (line[2],line[3])
                #mid_point =  (line[2] - line[0]) // 2 +  line[0] 


                #self.err = self.width // 2  -  mid_point
                #if -20 < self.err < 20 and -10 < line[3] - line[1] < 10:
                
                cv2.line(img, pt1, pt2, (0,0,255), 3)
       
        cv2.imshow("12", edges)

        return 


    def get_result(self, img): 
        self.height = img.shape[0]
        self.width = img.shape[1]
        self.wrap_offset = 20
     
        self.src_corners = [(162, 300), (621, 300), (785, 479), (0, 479)]
        self.dst_corners = [(0 + self.wrap_offset, 0), (785 - self.wrap_offset, 0), (785 - self.wrap_offset, self.height), (0 + self.wrap_offset, self.height)]
        # self.src_corners = [(270, 181), (512, 181), (785, 479), (0, 479)]
        # self.dst_corners = [(0 + self.wrap_offset, 0), (785 - self.wrap_offset, 0), (785 - self.wrap_offset, self.height), (0 + self.wrap_offset, self.height)]
        # self.src_corners = [(272, 157), (535, 157), (853, 479), (0, 479)]
        # self.dst_corners = [(0 + self.wrap_offset, 0), (853 - self.wrap_offset, 0), (853 - self.wrap_offset, self.height), (0 + self.wrap_offset, self.height)]
        self.M = cv2.getPerspectiveTransform(np.float32(self.src_corners), np.float32(self.dst_corners))
        self.wrap_img= self.perspective_transform(img, self.M)

        
        self.binary = self.preprocessing(self.wrap_img)

        self.left_fit, self.right_fit, self.out_img = self.find_line_fit(self.binary)
        self.left_fitx, self.right_fitx, self.ploty = self.get_fit_xy(self.binary, self.left_fit, self.right_fit)

        self.M = cv2.getPerspectiveTransform(np.float32(self.dst_corners), np.float32(self.src_corners))
        self.result = self.project_back(self.binary, img, self.left_fitx, self.right_fitx, self.ploty, self.M)


        self.distance_from_center, self.err = self.calculate_pos(self.binary,self.left_fitx, self.right_fitx, self.ploty)
       
        return self.distance_from_center, self.err,self.result



    def preprocessing(self,img):

        self.gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        self.blur = cv2.GaussianBlur(self.gray,(5,5),0)
        self.ret,self.thresh = cv2.threshold(self.blur,127,255,cv2.THRESH_BINARY)
    # thresh[0:int(height/3),0:width-1] = 0
       
        #cv2.imshow('123',self.thresh )

        # kernel = np.ones((1,13),np.uint8) 
        # dilation = cv2.dilate(self.thresh,kernel,iterations = 1)
        # cv2.imshow('1213',dilation )


        # contours,hierarchy = cv2.findContours(self.thresh,cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        # print(contours)
        return self.thresh

    def perspective_transform(self,img, M):
        self.img_size = (img.shape[1], img.shape[0])
        self.warped = cv2.warpPerspective(img, M, self.img_size, flags=cv2.INTER_LINEAR)

        
        return self.warped

    #滑动窗口
    def find_line_fit(self,img, nwindows=9, margin=50, minpix=50):
        self.histogram = np.sum(img[img.shape[0]//2:,:], axis=0)
        # Create an output image to draw on and  visualize the result
        self.out_img = np.dstack((img, img, img)) * 255
        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        self.midpoint = np.int(self.histogram.shape[0]/2)
        self.leftx_base = np.argmax(self.histogram[:self.midpoint])
        self.rightx_base = np.argmax(self.histogram[self.midpoint:]) + self.midpoint

        # Set height of windows                               
        self.window_height = np.int(img.shape[0]/nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        self.nonzero = img.nonzero()
        self.nonzeroy = np.array(self.nonzero[0])
        self.nonzerox = np.array(self.nonzero[1])
        # Current positions to be updated for each window
        self.leftx_current = self.leftx_base
        self.rightx_current = self.rightx_base
        # Create empty lists to receive left and right lane pixel indices
        self.left_lane_inds = []
        self.right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            self.win_y_low = img.shape[0] - (window+1)*self.window_height
            self.win_y_high = img.shape[0] - window*self.window_height
            self.win_xleft_low = self.leftx_current - margin
            self.win_xleft_high = self.leftx_current + margin
            self.win_xright_low = self.rightx_current - margin
            self.win_xright_high = self.rightx_current + margin
            # Draw the windows on the visualization image
            cv2.rectangle(self.out_img,(self.win_xleft_low,self.win_y_low),(self.win_xleft_high,self.win_y_high),
            (0,255,0), 2)
            cv2.rectangle(self.out_img,(self.win_xright_low,self.win_y_low),(self.win_xright_high,self.win_y_high),
            (0,255,0), 2)
            # Identify the nonzero pixels in x and y within the window
            self.good_left_inds = ((self.nonzeroy >= self.win_y_low) & (self.nonzeroy < self.win_y_high) &
            (self.nonzerox >= self.win_xleft_low) &  (self.nonzerox < self.win_xleft_high)).nonzero()[0]
            self.good_right_inds = ((self.nonzeroy >= self.win_y_low) & (self.nonzeroy < self.win_y_high) &
            (self.nonzerox >= self.win_xright_low) &  (self.nonzerox < self.win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            self.left_lane_inds.append(self.good_left_inds)
            self.right_lane_inds.append(self.good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(self.good_left_inds) > minpix:
                self.leftx_current = np.int(np.mean(self.nonzerox[self.good_left_inds]))
            if len(self.good_right_inds) > minpix:
                self.rightx_current = np.int(np.mean(self.nonzerox[self.good_right_inds]))

        # Concatenate the arrays of indices
        self.left_lane_inds = np.concatenate(self.left_lane_inds)
        self.right_lane_inds = np.concatenate(self.right_lane_inds)

        # Extract left and right line pixel positions
        self.leftx = self.nonzerox[self.left_lane_inds]
        self.lefty = self.nonzeroy[self.left_lane_inds]
        self.rightx = self.nonzerox[self.right_lane_inds]
        self.righty = self.nonzeroy[self.right_lane_inds]

        # to plot
        self.out_img[self.nonzeroy[self.left_lane_inds], self.nonzerox[self.left_lane_inds]] = [255, 0, 0]
        self.out_img[self.nonzeroy[self.right_lane_inds], self.nonzerox[self.right_lane_inds]] = [0, 0, 255]

        # Fit a second order polynomial to each
        if len(self.leftx):
            self.left_fit = np.polyfit(self.lefty, self.leftx, 2)
        if len(self.rightx):
            self.right_fit = np.polyfit(self.righty, self.rightx, 2)

       

      #  print ('left is empty:' + str(self.left_fit == []))
       # print ('right is empty:' + str(self.right_fit == []))
        #cv2.imshow('1', self.out_img)
        return self.left_fit, self.right_fit, self.out_img
    
    def calculate_pos(self,binary_warped,leftx, rightx, ploty):
        # Define y-value where we want radius of curvature
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )

    #     假设车道线方程为二次拟合曲线：x = a*y*y + b*y +c
        leftx = self.left_fit[0]*ploty**2 + self.left_fit[1]*ploty + self.left_fit[2]
        rightx = self.right_fit[0]*ploty**2 + self.right_fit[1]*ploty + self.right_fit[2]
            
        # ************************计算中心点**********************************
        # leftx为车道线集合中整幅图最前面的一个像素点 倒叙 image.shape = 375, 1242 所以最后一个点应为374
        lane_width = np.absolute(leftx[binary_warped.shape[0]-1] - rightx[binary_warped.shape[0]-1])


        # 3.7(m)/车道线宽像素点
        # 原图车道线像素宽度补偿
        lane_xm_per_pix = 3.7 / (lane_width*5)  
        #veh_pos = (((leftx[binary_warped.shape[0]-1] + rightx[binary_warped.shape[0]-1]) * lane_xm_per_pix) / 2.)
      
        self.veh_pos = int(((leftx[binary_warped.shape[0]-1] + rightx[binary_warped.shape[0]-1])) / 2)
            
        # 需要确定camera的位置 及坐标系
        #cen_pos = ((binary_warped.shape[1] * lane_xm_per_pix) / 2.)
        self.cen_pos = int((binary_warped.shape[1]) / 2)
        #print (cen_pos)
        #　为负说明中心线在右侧（车偏左）
        # 为正说明中心线在左侧（车偏右）
        # 0.06为KITTI中camera2坐标系决定补偿
        #distance_from_center = veh_pos - cen_pos +0.06
        self.distance_from_center = self.veh_pos - self.cen_pos

        self.tmp = int(((leftx[0] + rightx[0])) / 2)
        self.err = self.veh_pos - self.tmp
        # if err > 0 then means the lane tend to left, else tend to right
     

        # print(distance_from_center)
        return self.distance_from_center, self.err

    # Generate x and y values for plotting
    def get_fit_xy(self,img, left_fit, right_fit):
        self.ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
        self.left_fitx = left_fit[0]*self.ploty**2 + left_fit[1]*self.ploty + left_fit[2]
        self.right_fitx = right_fit[0]*self.ploty**2 + right_fit[1]*self.ploty + right_fit[2]

        return self.left_fitx, self.right_fitx, self.ploty

    def project_back(self,wrap_img, origin_img, left_fitx, right_fitx, ploty, M):

        self.warp_zero = np.zeros_like(wrap_img).astype(np.uint8)
        self.color_warp = np.dstack((self.warp_zero, self.warp_zero, self.warp_zero))

        # Recast the x and y points into usable format for cv2.fillPoly()
        self.pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        self.pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])

        self.pts = np.hstack((self.pts_left, self.pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(self.color_warp, np.int_([self.pts]), (0, 255, 0))

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        self.newwarp = self.perspective_transform(self.color_warp, M)
        # Combine the result with the original image
        self.result = cv2.addWeighted(origin_img, 1, self.newwarp, 0.3, 0)

        #cv2.putText(self.result,'Good Lane Keeping!',(10,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,200,0),2,cv2.LINE_AA)
        #cv2.imshow('s',self.result)
        return self.result




