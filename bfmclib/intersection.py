#!/usr/bin/env python

import networkx as nx

PosX = 0
PosY = 1
NegX = 2
NegY = 3


class crossing:
    def __init__(self, filename):
        self.G = nx.read_graphml(filename)

    # according to the path, decide turn left / right / go straight.
    def cross_hardcode(self, aNode, cNode):
        
        self.turn = self.turn_ori_cal(aNode, cNode)

        if self.turn == 0:
            self.speed = 0.3
            self.angle = 0
            print("Go straight")
        elif self.turn == 1:
            self.speed = 0.3
            self.angle = -12
            print("turn left")
        elif self.turn == 3:
            self.speed = 0.3
            self.angle = 14
            print("turn right")
        return self.speed, self.angle, self.cur_ori, self.fut_ori, self.turn

    """
    double loop PID control of steering
    :param pos0: target coordinte along the future orientation axis
    :param my_pos: my position
    :param my_prev_pos: last position
    :param my_inte_pos: integration position
    :param psi: yaw difference towards target yaw
    :param my_prev_psi: last psi
    :return: steering angle
    """
    def double_loop_pid(self, pos0, my_pos, my_inte_pos, my_prev_pos, psi, my_inte_psi, my_prev_psi):

        # err_pos = pos0 - my_pos
        err_pos = my_pos - pos0
        psi_offset = self.pid_controller(1.5, 0.0, 0.01, err_pos, my_prev_pos, 0.0)
        err_psi = psi_offset - psi
        print("err_pos: ", err_pos)
        print("err_psi: ", err_psi)
        print("psi_offset: ", psi_offset)
        print("psi: ", psi)
        steer_angle = self.pid_controller(5.0, 0.0, 10.0, err_psi, my_prev_psi, 0.0)
        return -steer_angle
        

    def turn_ori_cal(self, aNode, cNode):
        ax = self.G.node[aNode]['x']
        ay = self.G.node[aNode]['y']

        bNode = list(self.G.successors(aNode))[0]
        bx = self.G.node[bNode]['x']
        by = self.G.node[bNode]['y']

        # cNode = list(G.successors(bNode))
        cx = self.G.node[cNode]['x']
        cy = self.G.node[cNode]['y']
        print(ax, ay, bx, by, cx, cy)

        dX_ba = bx - ax
        dX_cb = cx - bx
        dY_ba = by - ay
        dY_cb = cy - by

        self.cur_ori = self.ori_cal(dY_ba, dX_ba)
        self.fut_ori = self.ori_cal(dY_cb, dX_cb)

        self.turn = (self.fut_ori - self.cur_ori) % 4 
        return self.turn


    # internal function
    def ori_cal(self, dy, dx):

        if abs(dy) > abs(dx):  # direction is along y axis
            if dy > 0:   # +y axis
                return PosY
            else:
                return NegY
        else:
            if dx > 0:
                return PosX
            else:
                return NegX
    
    # Return a boolean. True: The car have reached the node or the center of two nodes
    def is_reached_node(self, pos, tori, tnode1, tnode2=None):    # car position, target orientation, target node
        if tnode2 == None:   # reach the node
            tnode2 = tnode1
        else:   # reach the center of two nodes
            pass
        if pos:
            x_self = pos['x']
            y_self = pos['y']
            x_node = (self.G.node[tnode1]['x'] + self.G.node[tnode2]['x'])/2
            y_node = (self.G.node[tnode1]['y'] + self.G.node[tnode2]['y'])/2
            if tori == PosX and x_self > x_node:
                return True
            if tori == NegX and x_self < x_node:
                return True
            if tori == PosY and y_self > y_node:
                return True
            if tori == NegY and y_self < y_node:
                return True
            return False
        else:
            print("Error in is_reached_node, there is no position.")
            return False

    # calculate the current orientation based on node of path
    def orientation(self, my_node):
        ax = self.G.node[my_node]['x']
        ay = self.G.node[my_node]['y']

        bNode = list(self.G.successors(my_node))[0]
        bx = self.G.node[bNode]['x']
        by = self.G.node[bNode]['y']

        # print(ax, ay, bx, by)

        dX_ba = bx - ax
        dY_ba = by - ay

        self.cur_ori = self.ori_cal(dY_ba, dX_ba)
        print('cur ori: ', self.cur_ori)
        return self.cur_ori

    def pid_controller(self, P, I, D, Err, InteErr=0.0, PrevErr=0.0):

        output = P*Err + I*InteErr + D*(Err- PrevErr)
        return output
            





