#!/usr/bin/env python

"""
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""
from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight

# Our libs
from bfmclib.pid import PID
from bfmclib.lane_keeping import Keeper
from bfmclib.global_planning import planning
from bfmclib.trafficlight_color import TLcolor
from utils import inference

import rospy
import cv2
from time import sleep, time
import math
import _thread
import os

dir_path = os.path.dirname(os.path.realpath(__file__))
graph_path = os.path.join(dir_path, 'bfmclib/fixed_track.graphml')


# This line should be the first line in your program
rospy.init_node('ma=in_node', anonymous=True)

# Initialize necessary objects
cam = CameraHandler()
print("Camera loaded")

car = Controller()
print("Controller loaded")

sem = TrafficLight()
print("Traffic lights listener")

gps = Gps()
print("Gps loaded")

bno = BNO055()
print("BNO055 loaded")

od = inference.ObjectDetector()
print("Object detector loaded")

tlc = TLcolor()
print("Traffic light color checker loaded")

keeper = Keeper()
print("Keeper loaded")

pla = planning(graph_path)
print ("global planning loaded")

pid = PID(0.5, 0, 0.3)
pid2 = PID(50, 0, 10)
print ("PID loaded")

print("Select \"Frame preview\" window and press Q to exit")

sleep(1)

# Generate target path
begin_node = "86"
finish_node = "99"
path = pla.plan(begin_node, finish_node)
begin_node = "100"
finish_node = "116"
path += pla.plan(begin_node, finish_node)
begin_node = "117"
finish_node = "300"
path += pla.plan(begin_node, finish_node)
begin_node = "301"
finish_node = "196"
path += pla.plan(begin_node, finish_node)
begin_node = "197"
finish_node = "85"
path += pla.plan(begin_node, finish_node)

# dalete nodes in roundabout (for now)
del(path[path.index('302')])
del(path[path.index('303')])
del(path[path.index('304')])
del(path[path.index('305')])
del(path[path.index('306')])

# delete nodes in intersection (for now)
for index, val in enumerate(path):
    if val in pla.get_middle_list():
        del(path[index])
print ('path by global planning: {}'.format(path))
    

# Initialize parameters
pos = {'x':None, 'y':None}
flag = 0

PrevPosi = {'x': None, 'y': None}	# The gps is not updating all the time, we have to check it.
PrevGpsPosi = {'timestamp': None, 'coor': None}
my_position = {'x': None, 'y': None}

PrevTime = 0.0
speed = 0.3 
steering = 0

not_finished = False
direction = ''

err = 0
err2 = 0
lk_image = cam.getImage()
od_image = cam.getImage()

turnon_lk = True
turnon_od = True

timer_start = None
achieve_goal = False

exception_nodes = ['295','296','302','303','304','305','306','265','266','95','96','97']
tl_nodes = [['86','77'],['69','110','2'],['98','99','100','4'],['33','113','6']]


# The function for turning at intersection
def turn():
    global flag
    global base
    global speed
    global steering
    global my_position
    global direction

    tmp = False
    cur_node = pla.locate(my_position)
    target_node = path[0]
    cur_x, cur_y = pla.get_node(cur_node)
    target_x, target_y = pla.get_node(target_node)

    if flag == 0:
        base = bno.getYaw()
        # check direction
        if math.pi/4 < bno.getYaw() < 3*math.pi/4:
            if target_x - cur_x < 0:
                direction = 'left'
            else:
                direction = 'right'

        if -math.pi/4 < bno.getYaw() < math.pi/4:
            if target_y - cur_y > 0:
                direction = 'left'
            else:
                direction = 'right'

        if -3*math.pi/4 < bno.getYaw() < -math.pi/4:
            if target_x - cur_x > 0:
                direction = 'left'
            else:
                direction = 'right'

        if 3*math.pi/4 < bno.getYaw() < math.pi or -math.pi < bno.getYaw() < -3*math.pi/4:
            if target_y - cur_y < 0:
                direction = 'left'
            else:
                direction = 'right'

        if math.pi/4 < bno.getYaw() < 3*math.pi/4 or -3*math.pi/4 < bno.getYaw() < -math.pi/4:
            err_x = target_y - my_position['y']      
            flag = 1
        else:
            err_x = target_x - my_position['x']
            flag = 2

    elif flag == 2:
        err_x = target_x - my_position['x']
    elif flag == 1:
        err_x = target_y - my_position['y']

    # speed = 0.3
    steering = 0

    if direction == 'right' and -0.5 < err_x < 0.5:
        tmp = True
    elif direction == 'left' and -0.3 < err_x < 0.3:
        tmp = True

    if tmp:
        if direction == 'right':
            target = base - math.pi/2   # turn left
        elif direction == 'left':
            target = base + math.pi/2   # turn right

        err_yaw = target - bno.getYaw()
    
        # speed = 0.3
        steering = -err_yaw*60 
    
        if -0.005 < err_yaw < 0.005:
            return False
        else:
            return True

# The function for going straight forward at intersection or crosswalk
def follow(my_position, next_node):
    if my_position['x']:
        tolerant = 0.2
        target_x, target_y = pla.get_node(next_node)

        cur_x = float(my_position['x'])
        cur_y = float(my_position['y'])
        err = 0
        if target_y - cur_y < tolerant:
            if -3*math.pi/4 < bno.getYaw() < -math.pi/4:
                err = -bno.getYaw() - math.pi/2
            elif -math.pi/4 < bno.getYaw() < 3*math.pi/4:
                err = math.atan2((target_y - cur_y),(target_x - cur_x))
            else:   
                err = min(bno.getYaw() - math.pi, bno.getYaw() + math.pi )
        elif target_x - cur_x < tolerant:
            err = -math.atan2((target_x - cur_x),(target_y - cur_y))
        return err


# ###### Three threads are used ######
#   [0] - main thread
#   [1] - lane keeping thread
#   [2] - object detection thread
# ####################################


# [1] Thread for lane keeping
def lane_keeping_thread(name):
    global err
    global err2
    global lk_image

    while 1:
        err, err2, lk_image = keeper.get_result(cam.getImage())


# [2] Thread for object detection
# traffic signs
pedestrian_sign = False
pass_pedestrian_sign = True
stop_sign = False
pass_stop_sign = True
traffic_lights = False
pass_traffic_lights = True
parking_sign = False
sign_text = ''

# check traffic light color by cv2
roi = None

def object_detection_thread(name):
    # All objects:
    cls_dict = {0: 'car', 1: 'crosswalk', 2: 'highway_entrance', 3: 'highway_exit', 4: 'no-entry_road',
                5: 'one-way_road', 6: 'parking', 7: 'pedestrian', 8: 'pedestrian_doll', 9: 'priority',
                10: 'roadblock_left', 11: 'roadblock_right', 12: 'roundabout', 13: 'stop', 14: 'traffic_light'}
    global od_image
    global speed
    global pedestrian_sign
    global pass_pedestrian_sign
    global stop_sign
    global pass_stop_sign
    global traffic_lights
    global pass_traffic_lights
    global parking_sign
    global sign_text
    global timer_start
    global roi

    while 1:
        # print('++++++++++++++++++++++++++++++++++')
        if turnon_lk:
            od_image, od_cls, od_box = od.process_image(lk_image)
        else:
            od_image, od_cls, od_box = od.process_image(cam.getImage())
        # print('objects: {}'.format(od_cls))
        # print('boxes: {}'.format(od_box))

        if speed > 0:
            sign_text = ''
            for (cls, box) in zip(od_cls, od_box):
                w, h, m = box[2]-box[0], box[3]-box[1], (box[0]+box[2])/2
                # print('class: {}, w: {}, h: {}, m: {}'.format(cls_dict[cls], w, h, m))
                if cls == 1 and (w > 40 or h > 40) and not pedestrian_sign:
                    speed = 0.1
                    sign_text = 'Crosswalk sign detected! Slow down!'
                
                if cls == 6 and (w > 60 or h > 60) and od_cls.count(cls) == 2:
                    speed = 0.1
                    timer_start = time()
                    parking_sign = True
                
                if cls == 7 and (w > 75 or h > 125) and 400 < m < 470 and pass_pedestrian_sign:
                    speed = 0.0
                    timer_start = time()
                    pedestrian_sign = True
                    pass_pedestrian_sign = False
                    sign_text = 'Pedestrian detected! Stop!'
                
                if cls == 9 and (w > 60 or h > 60):
                    sign_text = 'Priority sign detected! You can pass!'
                
                if cls == 12 and (w > 60 or h > 60):
                    sign_text = 'Roundabout sign detected!'
                
                if cls == 13 and (w > 75 and h > 75) and pass_stop_sign:
                    speed = 0.0
                    timer_start = time()
                    stop_sign = True
                    pass_stop_sign = False
                    sign_text = 'Stop sign detected! Stop!'
                
                if cls == 14 and (w > 60 or h > 110):
                    sign_text = 'Traffic lights detected!'
                
                if cls == 14 and (w > 100 or h > 180) and pass_traffic_lights:
                    timer_start = time()
                    traffic_lights = True
                    pass_traffic_lights = False
                    roi = box


# Start additional threads
_thread.start_new_thread(lane_keeping_thread, ("Thread-1",))
_thread.start_new_thread(object_detection_thread, ("Thread-2",))


# [0] Main thread
while 1:
    # print ('---------------------------------------------------')
    turnon_lk = True

    # Localization
    PrevPosi['x'] = my_position['x']
    PrevPosi['y'] = my_position['y']
    gps_position = gps.getGpsData()
    if gps_position['coor']:
        if gps_position['coor'] != PrevGpsPosi['coor']: # When the gps data changes
            my_position['x'] = gps_position['coor'][0].real
            my_position['y'] = gps_position['coor'][0].imag
            PrevGpsPosi['coor'] = gps_position['coor']
        else:
            deltaX, deltaY = pla.localize_update(PrevTime, speed, bno.getYaw())
            my_position['x'] += deltaX
            my_position['y'] += deltaY
    PrevTime = rospy.get_time()	
    if pla.locate(my_position) not in pla.get_middle_list():
        cur_node = pla.locate(my_position)
    # print ('current node is ' + str(cur_node))

    # GPS
    if path:
        gps_steering = follow(my_position, path[0])
        # print (gps_steering)
    
    # Update the path
    if path:
        if cur_node == path[0]:
            del(path[0])
    else:
        achieve_goal = True
    
    # steering
    if cur_node in pla.get_entry_list() or not_finished:
        turnon_lk = False # turn off lane keeping
        cur_x, cur_y = pla.get_node(cur_node)
        t_x, t_y = pla.get_node(path[0])

        if -0.2 <cur_x - t_x < 0.2 or -0.2<cur_y - t_y < 0.2:
            steering = -pid2.get_steering(gps_steering)
            flag = 1
            not_finished = False
        else:
            not_finished = turn()
    else:
        flag = 0

    ctr_val = pid.get_steering(err)
    
    # Lane keeping mode
    if flag == 0:
        # speed = 0.3
        if cur_node in exception_nodes:
            turnon_lk = False # turn off lane keeping
            # print(gps_steering)
            steering = -gps_steering*25
        else:
            steering = ctr_val
    
    
    # ####################################################
    # Reaction towards traffic signs
    # print('current speed: {}'.format(speed))
    image_copy = od_image.copy()

    # print(bno.getPitch())
    if bno.getPitch() < -0.146:
        sign_text = 'Ramp deteced! Uphilling!'
    if bno.getPitch() > 0.146:
        sign_text = 'Ramp deteced! Downhilling!'
    

    if traffic_lights:
        roi_image = image_copy[int(roi[1]):int(roi[3]),int(roi[0]):int(roi[2])]
        tl_signal = tlc.recognize_tf(roi_image)
        # print('...Traffic light signal: {}'.format(tl_signal))
        sign_text = ''
        if tl_signal == 0:
            speed = 0.0
            timer_start = time()
            sign_text = 'Red light! Stop!'
        if tl_signal == 1:
            speed = 0.0
            timer_start = time()
            sign_text = 'Yellow light! Wait!'
        if tl_signal == 2:
            speed = 0.3
            sign_text = 'Green light! Go!'
            if time() - timer_start > 2:
                pass_traffic_lights = True
                traffic_lights = False
    
    
    if parking_sign:
        sign_text = 'Parking sign detected! You can park!'
        if time() - timer_start > 20:
            speed = 0.3
            parking_sign = False
    if stop_sign:
        if time() - timer_start > 5:
            sign_text = ''
            speed = 0.3
        if time() - timer_start > 7:
            pass_stop_sign = True
            stop_sign = False
    if pedestrian_sign:
        if time() - timer_start > 5:
            sign_text = ''
            speed = 0.3
        if time() - timer_start > 7:
            pass_pedestrian_sign = True
            pedestrian_sign = False
    
    if achieve_goal:
        sign_text = 'Target achieved! Engine stop!'
        speed = 0
        steering = 0
    
    car.drive(speed,steering)

    if sign_text:
        cv2.putText(image_copy,sign_text,(10,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2,cv2.LINE_AA)
    cv2.imshow("Driver View", image_copy)
    key = cv2.waitKey(1)
    if key == ord('q'):
        cv2.destroyAllWindows()
        break

print("Car stopped. \n END")
car.stop(0.0)
