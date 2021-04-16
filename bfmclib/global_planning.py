#!/usr/bin/env python

import networkx as nx
import rospy
import math

PosX = 0
PosY = 1
NegX = 2
NegY = 3


class planning:
    def __init__(self, filename):
        self.G = nx.read_graphml(filename)

        # nodes at the entrance of the crossing
        entry_list = [77, 81, 79, 45, 43, 41, 54, 50, 52,\
                            68, 72, 70, 2, 6, 4, 32, 34, 36, \
                            63, 59, 61, 14, 15, 16, 18, 23, 25, 27]
        self.ent_list = []
        for each in entry_list:
            self.ent_list.append(str(each))

        self.midle_nodes = ['82','83','84','48','46','47','57','56','55',\
                            '73','74','75','9','10','11','38','37','39',\
                                '64','65','66','19','20','21','28','29','30'] 

        # nodes at the entrance of cv
        cv_begin_list = [232, 49, 343, 199, 173, 149, 31]
        self.cv_entry_list = []
        for each in cv_begin_list:
            self.cv_entry_list.append(str(each))

        # nodes at the end of cv
        cv_finish_list = [423, 370, 342, 264, 229, 169, 196, 118]
        self.cv_end_list = []
        for each in cv_finish_list:
            self.cv_end_list.append(str(each))

        # nodes at the beginning of the roundabout
        round_begin_list = [301]
        self.round_entry_list = []
        for each in round_begin_list:
            self.round_entry_list.append(str(each))
        
        # nodes at the end of the roundabout
        round_finish_list = [232]
        self.round_end_list = []
        for each in round_finish_list:
            self.round_end_list.append(str(each))   
        

    """
    The planning function achieved by Dijkstra algorithm
    input: 
        start: the start node, in string
        target: the target node, in string
    return: 
        path: a list of node of path in string
    """

    def get_entry_list(self):
        return self.ent_list

    def get_middle_list(self):
        return self.midle_nodes

    def get_node(self,node):
        return self.G.node[node]['x'], self.G.node[node]['y']

    def plan(self, start, target):

        start = int(start)
        target = int(target)

        N = self.G.number_of_nodes()
        self.dist = [N+1] * (N+10)  # N+1 is inf, add the zeroth number because list starts with 0 in python
        self.q = range(1, N+1) # existing nodes in the graph
        self.prev = [N+1] * (N+10)  # previous point of all points

        self.dist[start] = 0 # init
        self.prev[start] = 0

        while self.q:
            # u := index of the smallest distance in q[]
            self.u = [] # index of the smallest points distance in q[]
            self.min_dist = N+1
            for index_of_point in self.q:
                if self.min_dist > self.dist[index_of_point]:
                    self.min_dist = self.dist[index_of_point]
                    self.u = [index_of_point]
                elif self.min_dist == self.dist[index_of_point]:
                    self.u.append(index_of_point)
            
            if self.min_dist == N+1:
                break
            if target in self.u:
                break
            
            self.q = [x for x in self.q if x not in self.u]   # remove u from q

            for border in self.u:
                for neighbor in list(self.G.successors(str(border))):
                    neighbor = int(neighbor)
                    temp = self.min_dist + 1
                    if temp < self.dist[neighbor]:
                        self.dist[neighbor] = temp
                        self.prev[neighbor] = border

        # path reconstruction
        last_point = target
        self.path = []
        while last_point != start:
            self.path.append(last_point)
            last_point = self.prev[last_point]
        self.path.append(start)
        self.path = list(reversed(self.path))

        for index, each in enumerate(self.path):
            self.path[index] = str(each)

        return self.path

    # locate the car at the nearest node
    def locate(self, pos):  
        if pos['x']:
            x = pos['x']
            y = pos['y']
            dmin = 1.0
            self.node = '0'
            for node_it in self.G._node:
                delx = self.G.node.get(node_it)['x'] - x
                dely = self.G.node.get(node_it)['y'] - y
                if (delx**2 + dely**2) < dmin:
                    dmin = (delx**2 + dely**2)
                    self.node = node_it
            if self.node == '0':
                print('The car is off the road!')
            else:
                return self.node

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
        # print('cur ori: ', self.cur_ori)
        return self.cur_ori


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

    def dist_from_cent(self, self_node, position):
        # position of node on central line
        xc = self.G.node[self_node]['x']
        yc = self.G.node[self_node]['y']
        try:
            # position of own
            xo = position['x']
            yo = position['y']
            self.ori = self.orientation(self_node)
            if self.ori == PosX:
                self.err = yo - yc
            elif self.ori == PosY:
                self.err = xc - xo
            elif self.ori == NegX:
                self.err = yc - yo
            elif self.ori == NegY:
                self.err = xo - xc
            else:
                print("can't get current orientation in function dist_from_cent")
            return self.err
        except:
            print("position is not acquired.")

    """
    Use IMU to compensate the localization
    return delta x and delta y.
    """
    def localize_update(self, PrevTime, vel, psi): 

        CurrTime = rospy.get_time()
        delta_t = CurrTime - PrevTime
        delta_x = vel * math.cos(psi) * delta_t
        delta_y = vel * math.sin(psi) * delta_t
        return delta_x, delta_y


# n1_succ_list = list(x.successors('1'))
# is_dot = x.get_edge_data('1', '60')['dotted']
# n1_item = x.node.get('1')
# n1_x = x.node.get('1')['x']