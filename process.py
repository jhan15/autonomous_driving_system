#!/usr/bin/env python

from bfmclib.lane_keeping import Keeper
from bfmclib.trafficlight_color import TLcolor
from utils import inference

from time import time


def lane_keeping_process(sender, receiver):
    keeper = Keeper()
    while 1:
        img = receiver.recv()

        try:
            err, err2, lk_image = keeper.get_result(img)
            sender.send([err, err2, lk_image])
        except:
            sender.send([0, 0, img])


def object_detection_process(sender, receiver):
    # traffic signs
    pedestrian = False
    pass_pedestrian = True
    stop_sign = False
    pass_stop_sign = True
    traffic_lights = False
    tl_signal = None
    pass_traffic_lights = True
    parking_sign = False
    priority_sign = False
    roundabout_sign = False
    crosswalk_sign = False

    h1 = None
    
    timer_start = None
    
    # All objects:
    cls_dict = {0: 'car', 1: 'crosswalk', 2: 'highway_entrance', 3: 'highway_exit', 4: 'no-entry_road',
                5: 'one-way_road', 6: 'parking', 7: 'pedestrian', 8: 'pedestrian_doll', 9: 'priority',
                10: 'roadblock_left', 11: 'roadblock_right', 12: 'roundabout', 13: 'stop', 14: 'traffic_light'}
    
    od = inference.ObjectDetector()
    print('od loaded in sub process')

    tlc = TLcolor()
    print("tlc loaded in sub process")
    
    while 1:
        obj_signals = [None] * 7
        obj_heights = [None] * 7
        tf_signal = None

        img, speed = receiver.recv()
        od_image, od_cls, od_box = od.process_image(img)

        for (cls, box) in zip(od_cls, od_box):
            w, h, m = box[2]-box[0], box[3]-box[1], (box[0]+box[2])/2
            # print('class: {}, w: {}, h: {}, m: {}'.format(cls_dict[cls], w, h, m))
            if cls == 1 and w > 30:
                obj_heights[0] = h
                obj_signals[0] = 1
            
            if cls == 7 and (w > 80 or h > 130) and 400 < m < 470:
                obj_heights[1] = h
                obj_signals[1] = 1

            if cls == 6 and w > 50:
                if obj_heights[2] is None:
                    obj_heights[2] = h
                    obj_signals[2] = 1
                elif w > obj_heights[2]:
                    obj_heights[2] = h

            if cls == 9 and w > 45:
                obj_heights[3] = h
                obj_signals[3] = 1
                
            if cls == 12 and w > 50:
                obj_heights[4] = h
                obj_signals[4] = 1
            
            if cls == 13 and w > 50:
                obj_heights[5] = h
                obj_signals[5] = 1
            
            if cls == 14 and w > 70:
                obj_heights[6] = h
                obj_signals[6] = 1
                roi = box
                roi_image = od_image[int(roi[1]):int(roi[3]),int(roi[0]):int(roi[2])]
                tf_signal = tlc.recognize_tf(roi_image)
        
        data = [obj_signals, obj_heights, tf_signal, od_image]
        
        sender.send(data)
