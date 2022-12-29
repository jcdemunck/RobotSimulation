import math

import cv2 as cv
import numpy as np

import os
import sys
import inspect

from XdockParams import W_DOCK, N_LANE, H_DOCK_LEGENDS, \
                        TIME_STEP_S, \
                        BLACK, \
                        get_distance_city_block, get_distance

from  ModelParameters import ModelParams as M
from  ModelParameters import get_model_params, get_log_filename

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
    def __init__(self):
        border_w          = 70
        border_h          = 100
        self.fig_width    = 1000
        self.fig_height   = border_h + int((self.fig_width-border_h) * M.H_FLOOR/M.W_FLOOR)
        if self.fig_height>800:
            self.fig_height = 800
            self.fig_width  = border_h + int((self.fig_height - border_h) * M.W_FLOOR/M.H_FLOOR)

        self.figure       = np.full((self.fig_height, self.fig_width, 3), 255, np.uint8)

        self.top_left     = (20, 50)
        self.bottom_right = (self.fig_width-(border_w-self.top_left[0]), self.fig_height-(border_h-self.top_left[1]))
        self.buffer_lanes  = dict()
        self.buffer_stores = dict()
        self.parkings      = dict()
        self.docks         = dict()
        pos_list           = []
        for dock in range(M.N_DOCK):
            self.docks[dock]    = Dock(dock)
            self.parkings[dock] = Parking(dock)
            pos_list           += self.parkings[dock].get_park_positions()
            for lane in range(N_LANE):
                self.buffer_lanes[dock, lane]  = BufferLane(dock, lane)
                pos_list                        += [self.buffer_lanes[dock, lane].get_grid_coords()]
            for store in range(M.N_BUFFER_STORE):
                self.buffer_stores[dock, store] = BufferStore(dock, store)
                pos_list                        += self.buffer_stores[dock, store].get_store_positions()
        self.grid_graph = self.__create_grid()
        self.path_table = GT.PathTable(self.grid_graph, pos_list, get_distance_city_block)

        self.robots = []
        for r in range(M.N_ROBOT):
            parking_pos = Position(self, dock=r%M.N_DOCK, parking=r//M.N_DOCK)
            if parking_pos is None:
                break
            color = None if r!=10 else (0, 0, 255)
            self.robots.append(Robot(parking_pos.w, parking_pos.h, parking_pos, color))

        # Legends parameters
        self.time_sec     = 0.
        self.n_trucks_in  = 0
        self.n_trucks_out = 0

    def time_step(self):
        for dock in range(M.N_DOCK):
            self.docks[dock].time_step(self)

            # time step each lane
            for lane in range(N_LANE):
                buffer_lane = self.buffer_lanes[dock, lane]
                buffer_lane.time_step()

            # Test if there is an unloaded roll container available at the dock and determine the least loaded lane (if any stores less than MAX_LANE_STORE roll containers)
            if self.docks[dock].roll_container_available():
                lane = self.get_best_available_lane(dock, output=False)
                if lane>=0:
                    buffer_lane = self.buffer_lanes[dock, lane]
                    if buffer_lane.can_be_loaded():
                        buffer_lane.reserve_store()
                        buffer_lane.store_roll_container(self.docks[dock].get_roll_container()) # load roll container from dock to lane

            # Test if there is a place on the dock for a roll container and if so, unload the best lane that can be unloaded
            if self.docks[dock].can_roll_container_be_stored():
                lane = self.get_best_available_lane(dock, output=True, loading=False)
                if lane>=0:
                    buffer_lane = self.buffer_lanes[dock, lane]
                    if buffer_lane.can_be_unloaded():
                        self.docks[dock].put_roll_container(buffer_lane.pickup_roll_container())

        for rob in self.robots:
            rob.time_step(self)

        self.time_sec += TIME_STEP_S

    def set_truck_list(self, truck_list):
        self.n_trucks_in  = len([t for t in truck_list if t.inbound])
        self.n_trucks_out = len([t for t in truck_list if not t.inbound])

        for dock in range(M.N_DOCK):
            self.docks[dock].set_truck_list(truck_list)

    def get_next_arrival(self):
        t_next = math.inf
        for dock in range(M.N_DOCK):
            t_arrive = self.docks[dock].truck_list[0].arrival
            t_next = min(t_next, t_arrive)

        return t_next

    def are_all_robots_idle(self):
        for rob in self.robots:
            if not rob.is_idle(): return False
        return True

    def get_nrc_incoming(self, dock):
        nrc = 0
        for lane in range(N_LANE):
            buffer_lane = self.buffer_lanes[dock, lane]
            if buffer_lane.lane_up:
                nrc += buffer_lane.get_n_stored()
        return nrc

    def get_incoming_roll_containers(self, dock):
        roll_containers = []
        for lane in range(N_LANE):
            buffer_lane = self.buffer_lanes[dock, lane]
            if buffer_lane.lane_up:
                roll_containers += buffer_lane.get_expected_roll_containers()

        roll_containers = [rol for rol in roll_containers if not rol.roll_container.scheduled]
        return sorted(roll_containers, key=lambda x: x[0])

    def get_best_available_lane(self, dock, output=True, loading=True):
        if loading:
            n_min    =  M.MAX_LANE_STORE+1
            lane_min = -1
            for lane in range(N_LANE):
                buffer_lane = self.buffer_lanes[dock, lane]
                if (buffer_lane.lane_up and output) or (not buffer_lane.lane_up and not output): continue
                if buffer_lane.n_store_reserved>=M.MAX_LANE_STORE: continue
                if buffer_lane.n_store_reserved<n_min:
                    lane_min = lane
                    n_min    = buffer_lane.n_store_reserved
            return lane_min
        else:
            n_max    =  0
            lane_max = -1
            for lane in range(N_LANE):
                buffer_lane = self.buffer_lanes[dock, lane]
                if (buffer_lane.lane_up and output) or (not buffer_lane.lane_up and not output): continue
                if buffer_lane.get_n_stored()>n_max:
                    lane_max = lane
                    n_max    = buffer_lane.get_n_stored()
            return lane_max

    def draw(self, draw_grid=False, draw_circulation=False):
        self.figure = np.full((self.fig_height, self.fig_width, 3), 255, np.uint8)
        self.figure = cv.rectangle(self.figure, self.top_left, self.bottom_right, BLACK, 2)

        if draw_grid:
            self.draw_grid()
        self.__draw_legends()
        for dock in range(M.N_DOCK):
            if dock>0:
                self.figure = cv.line(self.figure, self.pnt_from_coords(dock * W_DOCK, 0.),
                                                   self.pnt_from_coords(dock * W_DOCK, M.H_FLOOR), BLACK, 1)
            if draw_circulation:
                self.__draw_circulation(dock)
            for store in range(M.N_BUFFER_STORE):
                self.buffer_stores[dock, store].draw(self)

            self.docks[dock].draw(self)
            self.parkings[dock].draw(self)
            for lane in range(N_LANE):
                self.buffer_lanes[dock, lane].draw(self)

        for rob in self.robots:
            rob.draw(self)

    def pnt_from_coords(self, w, h):
        h = M.H_FLOOR - h
        x = int(self.top_left[0] + (w / M.W_FLOOR) * (self.bottom_right[0] - self.top_left[0]))
        y = int(self.top_left[1] + (h / M.H_FLOOR) * (self.bottom_right[1] - self.top_left[1]))
        return x, y

    def __create_grid(self):
        digraph = GT.DiGraph()

        for dock in range(M.N_DOCK):
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
            for store in range(M.N_BUFFER_STORE):
                buffer = self.buffer_stores[dock, store]
                w2, h2 = buffer.get_grid_coords(corner=2) # upper right
                q      = (w2, buffer.get_grid_coords(row=0, col=M.N_BUFFER_COL - 1)[1])

                # below buffer
                digraph.add_edge(buffer.get_grid_coords(corner=0), buffer.get_grid_coords(corner=1))
                # right of buffer, lower part
                digraph.add_edge(buffer.get_grid_coords(corner=1), q)

                for row in range(M.N_BUFFER_ROW):
                    p_new = buffer.get_grid_coords(row=row, col=M.N_BUFFER_COL - 1)
                    q_new = (w2,p_new[1])
                    digraph.add_edge(q_new, p_new) # left and right to first store
                    digraph.add_edge(p_new, q_new)
                    if row>0:
                        digraph.add_edge(q, q_new) # step up
                        q = q_new

                    for col in range(M.N_BUFFER_COL - 2, -1, -1):
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
        for dock in range(M.N_DOCK-1):
            # connect parkings
            digraph.add_edge(self.parkings[dock  ].get_grid_coords(corner=1), self.parkings[dock+1].get_grid_coords(corner=0))
            digraph.add_edge(self.parkings[dock+1].get_grid_coords(corner=3), self.parkings[dock  ].get_grid_coords(corner=2))

            # connect buffer stores
            for store in range(M.N_BUFFER_STORE):
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
            if parking>=self.parkings[dock].n_parc_col: return
            return self.parkings[dock].get_grid_coords(park=parking)

        if store>=0:
            if store>=M.N_BUFFER_STORE: return
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
        # Arrow to denote width of one dock
        h   = 1.01 * M.H_FLOOR
        pt1 = self.pnt_from_coords(0.      , h)
        pt2 = self.pnt_from_coords(W_DOCK/2, h)
        pt3 = self.pnt_from_coords(W_DOCK  , h)

        l = get_distance(pt1, pt2)
        self.figure = cv.arrowedLine(self.figure, pt2, pt1, (0, 0, 255), 2, tipLength=5 / l if l>0. else 0.)
        self.figure = cv.arrowedLine(self.figure, pt2, pt3, (0, 0, 255), 2, tipLength=5 / l if l>0. else 0.)

        # print width
        pt   = self.pnt_from_coords(W_DOCK/3, 1.01*h)
        text = f"{W_DOCK:5.1f} m"
        font = cv.FONT_HERSHEY_SIMPLEX
        self.figure = cv.putText(self.figure, text, pt, font, 0.4, BLACK)

        # Arrow denoting height of cross dock floor
        w   = 1.01 * M.W_FLOOR
        pt1 = self.pnt_from_coords(w, 0.       )
        pt2 = self.pnt_from_coords(w, M.H_FLOOR/2)
        pt3 = self.pnt_from_coords(w, M.H_FLOOR  )

        l = get_distance(pt1, pt2)
        self.figure = cv.arrowedLine(self.figure, pt2, pt1, (0, 0, 255), 2, tipLength=5 / l if l>0. else 0.)
        self.figure = cv.arrowedLine(self.figure, pt2, pt3, (0, 0, 255), 2, tipLength=5 / l if l>0. else 0.)

        # print height
        text = f"{M.H_FLOOR:5.1f} m"
        font = cv.FONT_HERSHEY_SIMPLEX
        pt   = self.pnt_from_coords(0.995*M.W_FLOOR, M.H_FLOOR/2)
        self.figure = cv.putText(self.figure, text, pt, font, 0.4, BLACK)

        header_time = f"t={(int(self.time_sec) // 3600 ) % 24:02d}:{(int(self.time_sec) // 60) % 60:02d}:{int(self.time_sec) % 60:02d}"
        pt = self.pnt_from_coords((M.N_DOCK-1.1)*W_DOCK, 1.01 * h)
        self.figure = cv.putText(self.figure, header_time, pt, font, 0.6, BLACK)

        if self.n_trucks_in>0 and self.n_trucks_out>0:
            w1 = M.W_FLOOR
            w2 = M.W_FLOOR + 0.7*W_DOCK
            h1 = H_DOCK_LEGENDS
            h2 = 0.
            h3 = (h1+h2)/2

            for out in [False, True]:
                if out:
                    frac = sum(self.docks[dock].get_n_trucks_todo(False) for dock in range(M.N_DOCK))/self.n_trucks_out
                    pt1  = self.pnt_from_coords(w1, h1)
                    pt2  = self.pnt_from_coords(w2-frac*(w2-w1), h3)
                else:
                    frac = sum(self.docks[dock].get_n_trucks_todo(True) for dock in range(M.N_DOCK))/self.n_trucks_in
                    pt1 = self.pnt_from_coords(w2, h3)
                    pt2 = self.pnt_from_coords(w1+frac*(w2-w1), h2)

                self.figure = cv.rectangle(self.figure, pt1, pt2, (100,100,255), -1)
                self.figure = cv.rectangle(self.figure, pt1, pt2, BLACK, 1)


    def __draw_circulation(self, dock):
        if dock<0 or M.N_DOCK<=dock: return

         # around parking buffer
        self.parkings[dock].draw_circulation(self)

        # around stores
        for store in range(M.N_BUFFER_STORE):
            self.buffer_stores[dock, store].draw_circulation(self)

    def get_incompletely_unloaded_trucks(self):
        return [t for dock in range(M.N_DOCK) for t in self.docks[dock].incomplete_unloaded]

    def get_n_roll_containers_in_store(self):
        return sum(self.buffer_stores[dock, buffer].get_n_stored() for buffer in range(M.N_BUFFER_STORE) for dock in range(M.N_DOCK))

    def get_n_roll_containers_in_lanes(self):
        return sum(self.buffer_lanes[dock, lane].get_n_stored() for lane in range(N_LANE) for dock in range(M.N_DOCK))

    def log_results(self):
        text = get_model_params()
        text += "\n\n\nResults:\n"
        t_list = self.get_incompletely_unloaded_trucks()
        text += f"NRC_in_buffer_store = {self.get_n_roll_containers_in_store():d} \n" +\
                f"NRC_in_buffer_lane  = {self.get_n_roll_containers_in_lanes():d} \n" +\
                f"NRC_not_unloaded    = {sum(len(t.truck_load) for t in t_list):d} \n"

        text += "Unloaded trucks: \n"
        if len(t_list)>0:
            text += t_list[0]._get_log_line(True)+'\n'
            for t in t_list:
                text += t._get_log_line(False)+'\n'
        with open(get_log_filename("Results"), "w") as fp:
            fp.write(text)

    def imshow(self, name):
        cv.imshow(name, self.figure)
