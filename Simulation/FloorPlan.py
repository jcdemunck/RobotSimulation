import math

import cv2 as cv
import numpy as np

import os
import sys
import inspect

from XdockParams import H_FLOOR, W_FLOOR, N_DOCK, W_DOCK, N_LANE, MAX_LANE_STORE, N_BUFFER_STORE, N_COMP_X, N_COMP_Y, \
                        TIME_STEP_S, \
                        BLACK, \
                        get_distance_city_block, get_distance

from Robot import Robot
from Dock import Dock
from Parking import Parking
from Position import Position
from BufferStore import BufferStore
from BufferLane import BufferLane


currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = currentdir[0:os.path.dirname(currentdir).rfind("\\")]
sys.path.insert(0, parentdir+"\\dks-ketenrekenmodel\\src\\krm\\core")
import GraphTools as GT

class FloorPlan:
    def __init__(self, n_robots=8):
        border_w          = 70
        border_h          = 100
        self.fig_width    = 1000
        self.fig_height   = border_h + int((self.fig_width-border_h) * H_FLOOR/W_FLOOR)
        if self.fig_height>800:
            self.fig_height = 800
            self.fig_width  = border_h + int((self.fig_height - border_h) * W_FLOOR/H_FLOOR)

        self.figure       = np.full((self.fig_height, self.fig_width, 3), 255, np.uint8)

        self.top_left     = (20, 50)
        self.bottom_right = (self.fig_width-(border_w-self.top_left[0]), self.fig_height-(border_h-self.top_left[1]))
        self.buffer_lanes  = dict()
        self.buffer_stores = dict()
        self.parkings      = dict()
        self.docks         = dict()
        pos_list           = []
        for dock in range(N_DOCK):
            self.docks[dock]    = Dock(dock)
            self.parkings[dock] = Parking(dock)
            pos_list           += self.parkings[dock].get_park_positions()
            for lane in range(N_LANE):
                self.buffer_lanes[dock, lane]  = BufferLane(dock, lane)
                pos_list                        += [self.buffer_lanes[dock, lane].get_grid_coords()]
            for store in range(N_BUFFER_STORE):
                self.buffer_stores[dock, store] = BufferStore(dock, store)
                pos_list                        += self.buffer_stores[dock, store].get_store_positions()
        self.grid_graph = self.__create_grid()
        self.path_table = GT.PathTable(self.grid_graph, pos_list, get_distance_city_block)

        self.robots = []
        for r in range(n_robots):
            parking_pos = Position(self, dock=r%N_DOCK, parking=r//N_DOCK)
            if parking_pos is None:
                break
            color = None if (r//N_DOCK)%2 else (0, 0, 255)
            self.robots.append(Robot(parking_pos.w, parking_pos.h, parking_pos, color))

        self.time_sec = 0.

    def get_robots(self, dock):
        return [rob for rob in self.robots if rob.default_pos.dock==dock]

    def time_step(self):
        for dock in range(N_DOCK):
            self.docks[dock].time_step(self)

            for lane in range(N_LANE):
                buffer_lane = self.buffer_lanes[dock, lane]
                buffer_lane.time_step()

                if buffer_lane.lane_up:
                    if buffer_lane.can_be_loaded() and self.docks[dock].roll_container_available():
                        buffer_lane.store_roll_container(self.docks[dock].get_roll_container())
                else:
                    if buffer_lane.can_be_unloaded() and self.docks[dock].can_roll_container_be_stored():
                        self.docks[dock].put_roll_container(buffer_lane.pickup_roll_container())

        for rob in self.robots:
            rob.time_step(self)

        self.time_sec += TIME_STEP_S

    def set_truck_list(self, truck_list):
        for dock in range(N_DOCK):
            self.docks[dock].set_truck_list(truck_list)

    def get_next_arrival(self, t_start):
        t_next = math.inf
        for dock in range(N_DOCK):
            for truck in self.docks[dock].truck_list:
                if truck.arrival<=t_start: continue
                t_next = min(t_next, truck.arrival)
                break

        return min(t_next, t_start)

    def are_all_robots_idle(self):
        for rob in self.robots:
            if not rob.is_idle(): return False
        return True

    def get_incoming_roll_containers(self, dock):
        roll_containers = []
        for lane in range(N_LANE):
            buffer_lane = self.buffer_lanes[dock, lane]
            if buffer_lane.lane_up:
                roll_containers += buffer_lane.get_expected_roll_containers()

        return sorted(roll_containers, key=lambda x: x[0])

    def get_best_available_lane(self, dock, output=True):
        n_min    =  MAX_LANE_STORE+1
        lane_min = -1
        for lane in range(N_LANE):
            buffer_lane = self.buffer_lanes[dock, lane]
            if (buffer_lane.lane_up and output) or (not buffer_lane.lane_up and not output): continue
            if buffer_lane.n_store_reserved>=MAX_LANE_STORE: continue
            if buffer_lane.n_store_reserved<n_min:
                lane_min = lane
                n_min    = buffer_lane.n_store_reserved
        return lane_min

    def draw(self, draw_grid=False, draw_circulation=False):
        self.figure = np.full((self.fig_height, self.fig_width, 3), 255, np.uint8)
        self.figure = cv.rectangle(self.figure, self.top_left, self.bottom_right, BLACK, 2)

        if draw_grid:
            self.draw_grid()
        self.__draw_legends()
        for dock in range(N_DOCK):
            if dock>0:
                self.figure = cv.line(self.figure, self.pnt_from_coords(dock * W_DOCK, 0.),
                                                   self.pnt_from_coords(dock * W_DOCK, H_FLOOR), BLACK, 1)
            if draw_circulation:
                self.__draw_circulation(dock)
            for store in range(N_BUFFER_STORE):
                self.buffer_stores[dock, store].draw(self)

            self.docks[dock].draw(self)
            self.parkings[dock].draw(self)
            for lane in range(N_LANE):
                self.buffer_lanes[dock, lane].draw(self)

        for rob in self.robots:
            rob.draw(self)

    def pnt_from_coords(self, w, h):
        h = H_FLOOR - h
        x = int(self.top_left[0] + (w / W_FLOOR) * (self.bottom_right[0] - self.top_left[0]))
        y = int(self.top_left[1] + (h / H_FLOOR) * (self.bottom_right[1] - self.top_left[1]))
        return x, y

    def __create_grid(self):
        digraph = GT.DiGraph()

        for dock in range(N_DOCK):
            park      = self.parkings[dock]
            p_park_0  = park.get_grid_coords(corner=0) # lower left
            p_park_1  = park.get_grid_coords(corner=1) # lower right
            p_park_2  = park.get_grid_coords(corner=2) # upper right
            p_park_3  = park.get_grid_coords(corner=3) # upper left

            # edges around parking
            path_right = [p_park_0, p_park_1] +\
                         [coord for coord in park.coord_dict.values()] +\
                         [self.buffer_lanes[dock, lane].get_grid_coords() for lane in range(N_LANE)]
            path_right.sort(key = lambda wh: wh[0])  # origins of points below and above path below parking to the right

            p_old = path_right[0]  # left most point
            h     = p_old[1]
            digraph.add_edge(p_park_3, p_old) # down left of parking
            for p_orig in path_right[1:-1]:
                p = (p_orig[0],h)
                digraph.add_edge(p_old, p) # step to right
                if p_orig[1]>h:
                    digraph.add_edge(p_orig, p)  # step down
                elif p_orig[1]<h:
                    digraph.add_edge(p_orig, p)  # step down
                    digraph.add_edge(p, p_orig)  # step up
                p_old = p

            digraph.add_edge(p_old, p_park_1)    # last step to the right (lower right parking corner)
            digraph.add_edge(p_park_1, p_park_2) # up right of parking

            p_left_most_buffer_coords  = self.buffer_stores[(dock,0)].get_grid_coords(corner=0)
            p_right_most_buffer_coords = self.buffer_stores[(dock,0)].get_grid_coords(corner=1)
            path_left = [p_left_most_buffer_coords] + [coord for coord in park.coord_dict.values()] + [p_right_most_buffer_coords]
            path_left = sorted(path_left,key = lambda wh: wh[0], reverse=True) # origins of points below and above path above parking to the left

            p_old = p_park_2 # right most point
            h     = p_old[1]
            for p_orig in path_left[1:-1]:
                p = (p_orig[0],h)
                digraph.add_edge(p_old, p)   # step to left
                digraph.add_edge(p, p_orig)  # step downwards
                p_old = p

            digraph.add_edge(p_old   , p_park_3)  # last step to the left
            digraph.add_edge(p_left_most_buffer_coords, p_park_3) # leftmost down
            digraph.add_edge(p_park_2, p_right_most_buffer_coords) # rightmost up

            # edges around each buffer store
            for store in range(N_BUFFER_STORE):
                buffer = self.buffer_stores[dock, store]
                w2, h2 = buffer.get_grid_coords(corner=2) # upper right
                q      = (w2, buffer.get_grid_coords(row=0, col=N_COMP_X-1)[1])

                # below buffer
                digraph.add_edge(buffer.get_grid_coords(corner=0), buffer.get_grid_coords(corner=1))
                # right of buffer, lower part
                digraph.add_edge(buffer.get_grid_coords(corner=1), q)

                for row in range(N_COMP_Y):
                    p_new = buffer.get_grid_coords(row=row, col=N_COMP_X-1)
                    q_new = (w2,p_new[1])
                    digraph.add_edge(q_new, p_new) # left and right to first store
                    digraph.add_edge(p_new, q_new)
                    if row>0:
                        digraph.add_edge(q, q_new) # step up
                        q = q_new

                    for col in range(N_COMP_X-2, -1, -1):
                        p_old = buffer.get_grid_coords(row=row, col=col)
                        digraph.add_edge(p_old, p_new)     # left and right to store at column col
                        digraph.add_edge(p_new, p_old)
                        p_new = p_old
                # right of buffer, upper part
                digraph.add_edge(q, buffer.get_grid_coords(corner=2))
                # above buffer
                digraph.add_edge(buffer.get_grid_coords(corner=2), buffer.get_grid_coords(corner=3))
                # left of buffer
                digraph.add_edge(buffer.get_grid_coords(corner=3), buffer.get_grid_coords(corner=0))

                #connect buffers
                if store>0:
                    buffer_old = self.buffer_stores[(dock,store-1)]
                    digraph.add_edge(buffer_old.get_grid_coords(corner=2), buffer.get_grid_coords(corner=1))
                    digraph.add_edge(buffer.get_grid_coords(corner=0), buffer_old.get_grid_coords(corner=3))


        # connect dock grids
        for dock in range(N_DOCK-1):
            # connect parkings
            digraph.add_edge(self.parkings[dock  ].get_grid_coords(corner=1), self.parkings[dock+1].get_grid_coords(corner=0))
            digraph.add_edge(self.parkings[dock+1].get_grid_coords(corner=3), self.parkings[dock  ].get_grid_coords(corner=2))

            # connect buffer stores
            for store in range(N_BUFFER_STORE):
                buffer  = self.buffer_stores[(dock  , store)]
                buffer1 = self.buffer_stores[(dock+1, store)]
                digraph.add_edge(buffer .get_grid_coords(corner=1), buffer1.get_grid_coords(corner=0))
                digraph.add_edge(buffer1.get_grid_coords(corner=3), buffer .get_grid_coords(corner=2))

        return digraph

    def draw_grid(self):
        color = (100, 100, 100)
        for e in self.grid_graph.get_edge_list():
            pt1 = self.pnt_from_coords(*e[0])
            pt2 = self.pnt_from_coords(*e[1])
            l   = get_distance(pt1, pt2)
            self.figure = cv.arrowedLine(self.figure, pt1, pt2, color, 1, tipLength=5/l if l>0. else 0.)
            self.figure = cv.circle(self.figure, pt1, 3, color, -1)
            self.figure = cv.circle(self.figure, pt2, 3, color, -1)

    def get_item_coords(self, dock, buffer_lane=-1, parking=-1, store=-1, row=-1, col=-1):
        if buffer_lane>=0:
            if buffer_lane>N_LANE: return
            return self.buffer_lanes[dock, buffer_lane].get_grid_coords()

        if parking>=0:
            if parking>=self.parkings[dock].n_parc_spot: return
            return self.parkings[dock].get_grid_coords(park=parking)

        if store>=0:
            if store>=N_BUFFER_STORE: return
            return self.buffer_stores[dock, store].get_grid_coords(row=row, col=col)

    def get_shortest_path(self, pos1, pos2):
        coords1 = pos1.get_coords()
        coords2 = pos2.get_coords()
        return self.path_table.get_path(coords1, coords2)

    def draw_path(self, path, color=(255,0,0)):
        p1 = path[0]
        for p in path:
            self.figure = cv.line(self.figure, self.pnt_from_coords(*p1), self.pnt_from_coords(*p), color, 2)
            p1 = p

    def __draw_legends(self):
        w1 = 0.
        w2 = W_DOCK/2
        w3 = W_DOCK
        h  = 1.01 * H_FLOOR

        pt1 = self.pnt_from_coords(w1, h)
        pt2 = self.pnt_from_coords(w2, h)
        pt3 = self.pnt_from_coords(w3, h)

        self.figure = cv.arrowedLine(self.figure, pt2, pt1, (0, 0, 255), 2)
        self.figure = cv.arrowedLine(self.figure, pt2, pt3, (0, 0, 255), 2)

        pt   = self.pnt_from_coords(W_DOCK/3, 1.01*h)
        text = f"{W_DOCK:5.1f} m"
        font = cv.FONT_HERSHEY_SIMPLEX
        self.figure = cv.putText(self.figure, text, pt, font, 0.6, BLACK)

        header_time = f"t={int(self.time_sec) // 3600:2d}:{(int(self.time_sec) // 60) % 60:2d}:{int(self.time_sec) % 60:02d}"
        pt = self.pnt_from_coords((N_DOCK-1)*W_DOCK, 1.01 * h)
        self.figure = cv.putText(self.figure, header_time, pt, font, 0.6, BLACK)

    def __draw_circulation(self, dock):
        if dock<0 or N_DOCK<=dock: return

         # around parking buffer
        self.parkings[dock].draw_circulation(self)

        # around stores
        for buffer in range(N_BUFFER_STORE):
            for store in range(N_BUFFER_STORE):
                self.buffer_stores[dock, store].draw_circulation(self)

    def imshow(self, name):
        cv.imshow(name, self.figure)
