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
        img, speed = receiver.recv()
        od_image, od_cls, od_box = od.process_image(img)

        for (cls, box) in zip(od_cls, od_box):
            w, h, m = box[2]-box[0], box[3]-box[1], (box[0]+box[2])/2
            # print('class: {}, w: {}, h: {}, m: {}'.format(cls_dict[cls], w, h, m))
            if cls == 1 and (w > 45 or h > 45):
                if not crosswalk_sign:
                    timer_start = time()
                    crosswalk_sign = True
            
            if cls == 7 and (w > 80 or h > 130) and 400 < m < 470:
                if not pedestrian and pass_pedestrian:
                    timer_start = time()
                    crosswalk_sign = False
                    pedestrian = True
                    pass_pedestrian = False

            if cls == 6 and (w > 65 or h > 65) and od_cls.count(cls) == 2:
                if not parking_sign:
                    timer_start = time()
                    parking_sign = True

            if cls == 9 and (w > 75 or h > 75):
                if not priority_sign:
                    timer_start = time()
                    priority_sign = True
                
            if cls == 12 and (w > 70 or h > 70):
                if not roundabout_sign:
                    timer_start = time()
                    roundabout_sign = True
            
            if cls == 13 and (w > 70 and h > 70):
                if not stop_sign and pass_stop_sign:
                    timer_start = time()
                    stop_sign = True
                    pass_stop_sign = False
            
            if cls == 14 and (w > 95 or h > 175):
                if not traffic_lights:
                    timer_start = time()
                    pass_traffic_lights = False
                
                if not pass_traffic_lights:
                    if tl_signal != 2:
                        timer_start = time()
                    
                    traffic_lights = True
                    roi = box
                    roi_image = od_image[int(roi[1]):int(roi[3]),int(roi[0]):int(roi[2])]
                    tl_signal = tlc.recognize_tf(roi_image)
                    # print('...Traffic light signal: {}'.format(tl_signal))
        
        if crosswalk_sign and not pedestrian and time()-timer_start > 10:
            crosswalk_sign = False
        
        if pedestrian and time()-timer_start > 8:
            pedestrian = False
        if not pass_pedestrian and time()-timer_start > 10:
            pass_pedestrian = True
        
        if priority_sign and time()-timer_start > 2:
            priority_sign = False
        
        if roundabout_sign and time()-timer_start > 2:
            roundabout_sign = False
        
        if stop_sign and time()-timer_start > 4:
            stop_sign = False
        if not pass_stop_sign and time()-timer_start > 6:
            pass_stop_sign = True
        
        if tl_signal == 2 and time()-timer_start > 2:
            traffic_lights = False
            pass_traffic_lights = True
            tl_signal = None
        
        if parking_sign and time()-timer_start > 10:
            parking_sign = False
        
        data = [crosswalk_sign, pedestrian, priority_sign, roundabout_sign,
                stop_sign, tl_signal, parking_sign, od_image]
        
        sender.send(data)
