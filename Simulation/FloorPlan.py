import cv2 as cv
import numpy as np

import os
import sys
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = currentdir[0:os.path.dirname(currentdir).rfind("\\")]
sys.path.insert(0, parentdir+"\\dks-ketenrekenmodel\\src\\krm\\core")
import GraphTools as GT


EPS   = 1.e-6
WHITE = (255,255,255)
BLACK = (0,0,0)

H_COMPARTMENT  = 0.9
W_COMPARTMENT  = 0.9
N_COMP_X       = 4 ## 3
N_COMP_Y       = 12
N_BUFFER_STORE = 1 ##2
W_BUFFER_STORE = N_COMP_X*W_COMPARTMENT
H_BUFFER_STORE = N_COMP_Y*H_COMPARTMENT


H_FRONT = 2.0
H_REAR  = 2.0

W_DOWN         = 0.8
W_MID          = 0.##0.5
W_UP           = 0.8
W_DOCK         = W_DOWN + N_BUFFER_STORE*W_BUFFER_STORE + W_MID +W_UP

H_PARK         = 2.0
W_PARK_PLACE   = 1.1
H_RIGHT        = 2.0
H_LEFT         = 2.0
H_MANEUVER     = H_RIGHT + H_PARK + H_LEFT

N_LANE         =  4   # For each dock: two up and two down
W_LANE         =  1.1
H_LANE         = 18.
MAX_LANE_STORE = 24
H_LANE_STORE   = 0.9

N_DOCK     =  4

W_FLOOR    = N_DOCK * W_DOCK
H_FLOOR    = H_FRONT + H_LANE + H_MANEUVER + N_COMP_Y * H_COMPARTMENT + H_REAR


H_ROLL_CONTAINER = 0.7
W_ROLL_CONTAINER = 0.7
TIME_ROLL_CONTAINER_LOAD = 2.5 # >= H_ROLL_CONTAINER/BUFFER_LANE_SPEED

W1_ROBOT = 0.3
W2_ROBOT = 0.9
H1_ROBOT = 0.7
H2_ROBOT = 0.4

TIME_STEP_S       =  0.3
ROBOT_SPEED       =  1.0  # (1.2) m/s
ROBOT_LOAD_TIME   = 5. # [s]
ROBOT_UNLOAD_TIME = 5. # [s]
BUFFER_LANE_SPEED =  0.3

_CO_SCALE = 1000.
def round_coord(co):
    return int(_CO_SCALE*co+0.5)/_CO_SCALE
def round_coords(co):
    return int(_CO_SCALE*co[0]+0.5)/_CO_SCALE, int(_CO_SCALE*co[1]+0.5)/_CO_SCALE

def get_distance(p1, p2):
    return np.sqrt( (p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) )

def get_distance_city_block(p1, p2):
    return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])

def get_path_len(path):
    p_len = 0.
    if len(path)>1:
        p_old = path[0]
        for p in path:
            p_len += get_distance_city_block(p, p_old)
            p_old  = p

    return p_len

def get_orientation(fr, to):
    if abs(fr[1]-to[1])<EPS:  # segment horizontal
        return 0 if to[0]>fr[0] else 2
    else:
        return 1 if to[1]>fr[1] else 3

from Robot import Robot
from Dock import Dock
from Parking import Parking
from BufferStore import BufferStore
from BufferLane import BufferLane

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
        self.header_text  = ""

        self.top_left     = (20, 50)
        self.bottom_right = (self.fig_width-(border_w-self.top_left[0]), self.fig_height-(border_h-self.top_left[1]))
        self.buffer_lanes  = dict()
        self.buffer_stores = dict()
        self.parkings      = dict()
        self.docks         = dict()
        for dock in range(N_DOCK):
            self.docks[dock]    = Dock(dock)
            self.parkings[dock] = Parking(dock)
            for lane in range(N_LANE):
                self.buffer_lanes[(dock, lane)]  = BufferLane(dock, lane)
            for store in range(N_BUFFER_STORE):
                self.buffer_stores[(dock,store)] = BufferStore(dock, store)

        self.grid_graph = self.__create_grid()

        self.robots = []
        for r in range(n_robots):
            dock = r%N_DOCK
            spot = self.parkings[dock].get_first_empty_spot(self)
            if spot is None:
                break

            self.parkings[dock].park(spot)
            self.robots.append(Robot(spot.w, spot.h, 3))

    def get_robots_idle(self):
        return [rob for rob in self.robots if len(rob.task_list)==0]

    def time_step(self):
        for dock in range(N_DOCK):
            for lane in range(N_LANE):
                self.buffer_lanes[(dock, lane)].time_step()

            # Move trolleys from (unloading) truck to input buffer lane(s)
            if self.docks[dock].get_nrc_input()>0:
                for lane in range(N_LANE):
                    buffer_lane = self.buffer_lanes[(dock, lane)]
                    if buffer_lane.lane_up and buffer_lane.can_be_loaded():
                        buffer_lane.store_roll_container(self.docks[dock].rc_unloading.pop())

        for rob in self.robots:
            rob.time_step()

    def get_incoming_roll_containers(self, dock):
        roll_containers = []
        for lane in range(N_LANE):
            buffer_lane = self.buffer_lanes[(dock, lane)]
            if not buffer_lane.lane_up: continue
            roll_containers += buffer_lane.get_expected_roll_containers()

        return sorted(roll_containers, key=lambda x: x[0])

    def start_unloading_truck(self, dock, roll_containers):
        self.docks[dock].start_unloading(roll_containers)

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
                self.buffer_stores[(dock, store)].draw(self)

            self.docks[dock].draw(self)
            self.parkings[dock].draw(self)
            for lane in range(N_LANE):
                self.buffer_lanes[(dock, lane)].draw(self)

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
                         [self.buffer_lanes[(dock, lane)].get_grid_coords() for lane in range(N_LANE)]
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

            digraph.add_edge(p_old, path_right[-1]) # last step to the right (lower right parking corner)
            digraph.add_edge(path_right[-1], p_park_2) # up right of parking

            p_left_most_buffer_coords  = self.buffer_stores[(dock,0               )].get_lowest_row_coords(left=True )
            p_right_most_buffer_coords = self.buffer_stores[(dock,N_BUFFER_STORE-1)].get_lowest_row_coords(left=False)
            path_left = [coord for coord in park.coord_dict.values()] + \
                        [self.buffer_stores[(dock,store)].get_lowest_row_coords(left=True) for store in range(1,N_BUFFER_STORE)] +\
                        [p_left_most_buffer_coords, p_right_most_buffer_coords]

            path_left  = sorted(path_left,key = lambda wh: wh[0], reverse=True) # origins of points below and above path above parking to the left

            p_old = p_park_2 # right most point
            h     = p_old[1]
            for p_orig in path_left[1:-1]:
                p = (p_orig[0],h)
                digraph.add_edge(p_old, p)   # step to left
                if p_orig[1]<h:
                    digraph.add_edge(p, p_orig)
                else:
                    digraph.add_edge(p_orig, p)
                p_old = p

            digraph.add_edge(p_old   , p_park_3)  # last step to the left
            digraph.add_edge(p_park_2, p_right_most_buffer_coords) # rightmost up

            # edges around each buffer store
            for store in range(N_BUFFER_STORE):
                buffer = self.buffer_stores[(dock,store)]
                w2, h2 = buffer.get_grid_coords(corner=2)
                q      = (w2, buffer.get_grid_coords(row=0, col=N_COMP_X-1)[1])
                for row in range(N_COMP_Y):
                    p_new = buffer.get_grid_coords(row=row, col=N_COMP_X-1)
                    q_new     = (w2,p_new[1])
                    digraph.add_edge(q_new, p_new) # left and right to first store
                    digraph.add_edge(p_new, q_new)
                    if row>0:
                        if store==N_BUFFER_STORE-1:
                            digraph.add_edge(q, q_new) # step up
                        else:
                            digraph.add_edge(q_new, q)  # step down
                        q = q_new

                    for col in range(N_COMP_X-2, -1, -1):
                        p_old = buffer.get_grid_coords(row=row, col=col)
                        digraph.add_edge(p_old, p_new)     # left and right to store at column col
                        digraph.add_edge(p_new, p_old)
                        p_new = p_old

                # Above buffer
                if store==N_BUFFER_STORE-1:
                    digraph.add_edge(q, (w2,h2)) # step up
                else:
                    digraph.add_edge((w2,h2), q)  # step down
                digraph.add_edge((w2,h2), buffer.get_grid_coords(corner=3))
                digraph.add_edge(buffer.get_grid_coords(corner=3), buffer.get_grid_coords(corner=0))

        # connect dock grids
        for dock in range(N_DOCK-1):
            digraph.add_edge(self.parkings[dock  ].get_grid_coords(corner=1), self.parkings[dock+1].get_grid_coords(corner=0))
            digraph.add_edge(self.parkings[dock+1].get_grid_coords(corner=3), self.parkings[dock  ].get_grid_coords(corner=2))

            digraph.add_edge(self.buffer_stores[(dock+1,0)               ].get_grid_coords(corner=3),
                             self.buffer_stores[(dock  ,N_BUFFER_STORE-1)].get_grid_coords(corner=2))

        return digraph

    def draw_grid(self):
        color = (100, 100, 100)
        for e in self.grid_graph.get_edge_list():
            pt1 = self.pnt_from_coords(*e[0])
            pt2 = self.pnt_from_coords(*e[1])
            l   = np.sqrt((pt1[0]-pt2[0])*(pt1[0]-pt2[0]) + (pt1[1]-pt2[1])*(pt1[1]-pt2[1]))
            self.figure = cv.arrowedLine(self.figure, pt1, pt2, color, 2, tipLength=5/l if l>0. else 0.)
            self.figure = cv.circle(self.figure, pt1, 3, color, -1)
            self.figure = cv.circle(self.figure, pt2, 3, color, -1)

    def get_item_coords(self, dock, buffer_lane=-1, parking=-1, store=-1, row=-1, col=-1):
        if buffer_lane>=0:
            if buffer_lane>N_LANE: return
            return self.buffer_lanes[(dock, buffer_lane)].get_grid_coords()

        if parking>=0:
            if parking>=self.parkings[dock].n_parc_spot: return
            return self.parkings[dock].get_grid_coords(park=parking)

        if store>=0:
            if store>=N_BUFFER_STORE: return
            return self.buffer_stores[(dock, store)].get_grid_coords(row=row, col=col)

    def get_shortest_path(self, pos1, pos2):
        coords1 = pos1.get_coords()
        coords2 = pos2.get_coords()
        return self.grid_graph.get_shortest_path(coords1, coords2, get_distance_city_block)

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
        self.figure = cv.putText(self.figure, text, pt, font, 0.7, BLACK)

        if type(self.header_text)==str:
            pt = self.pnt_from_coords((N_DOCK-1)*W_DOCK, 1.01 * h)
            self.figure = cv.putText(self.figure, self.header_text, pt, font, 0.7, BLACK)

    def __draw_circulation(self, dock):
        if dock<0 or N_DOCK<=dock: return

         # around parking buffer
        self.parkings[dock].draw_circulation(self)

        # around stores
        for buffer in range(N_BUFFER_STORE):
            for store in range(N_BUFFER_STORE):
                self.buffer_stores[(dock, store)].draw_circulation(self)

    def imshow(self, name):
        cv.imshow(name, self.figure)
