#!/usr/bin/env python

import cv2
import numpy as np


class TLcolor:
    def __init__(self, h_main=[0,30,60], h_max=179, h_thre=5):
        self.h_main = h_main
        self.h_max = h_max
        self.h_thre = h_thre
        self.tf_light = None
        self.s = (120, 255)
        self.v = (220, 255)
        self.hs = []
        self.color_bound = []

        self.initialize_hs()
        self.initialize_cb()
    
    def initialize_hs(self):
        for h in self.h_main:
            self.hs += self.generate_h_range(h)
    
    def initialize_cb(self):
        for h in self.hs:
            self.color_bound.append(
                ([h[0],self.s[0],self.v[0]],
                 [h[1],self.s[1],self.v[1]])
            )
    
    def generate_h_range(self, h):
        h_range = []
        h_l = h - self.h_thre
        h_u = h + self.h_thre
        if h_l < 0:
            h_l = self.h_max + h_l + 1
            h_range.append((0,h_u))
            h_range.append((h_l,self.h_max))
        elif h_u > self.h_max:
            h_u = h_u - self.h_max - 1
            h_range.append((0,h_u))
            h_range.append((h_l,self.h_max))
        else:
            h_range.append((h_l,h_u))
        
        return h_range
    
    def recognize_tf(self, image):
        # image = cv2.resize(image, (100,200))
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        mask = []
        for (lr, up) in self.color_bound:
            lr = np.array(lr, dtype='uint8')
            up = np.array(up, dtype='uint8')
            mask.append(cv2.inRange(hsv_image, lr, up))
        mask = sum(mask)
        seg_image = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        # bgr_seg_image = cv2.cvtColor(seg_image, cv2.COLOR_HSV2BGR)

        h = seg_image.T[0].copy()
        v = seg_image.T[2].copy()
        h[(h<self.hs[0][1])&(v>0)] += self.h_max + 1
        h_avg = h[v>0].mean()
        
        for i, h in enumerate(self.h_main):
            if h == 0:
                h = self.h_max + 1
            if h-self.h_thre <= h_avg <= h+self.h_thre:
                self.tf_light = i
                break
        
        return self.tf_light
