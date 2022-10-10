import cv2 as cv
import numpy as np

import os
import sys
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = currentdir[0:os.path.dirname(currentdir).rfind("\\")]
sys.path.insert(0, parentdir+"\\dks-ketenrekenmodel\\src\\krm\\core")
import GraphTools as GT


WHITE = (255,255,255)
BLACK = (0,0,0)

H_COMPARTMENT  = 1.0
W_COMPARTMENT  = 1.0
N_COMP_X       = 6 ## 3
N_COMP_Y       = 9
N_BUFFER_STORE = 1 ##2
W_BUFFER_STORE = N_COMP_X*W_COMPARTMENT
H_BUFFER_STORE = N_COMP_Y*H_COMPARTMENT


H_FRONT = 2.0
H_REAR  = 2.0

W_DOWN         = 1.5
W_MID          = 2.0
W_UP           = 1.5
W_DOCK         = W_DOWN + N_BUFFER_STORE*W_BUFFER_STORE + W_MID +W_UP

H_PARK         = 2.0
W_PARK_PLACE   = 1.4
H_RIGHT        = 2.0
H_LEFT         = 2.0
H_MANEUVER     = H_RIGHT + H_PARK + H_LEFT

N_LANE         =  4   # For each dock: two up and two down
W_LANE         =  1.5
H_LANE         = 18.
MAX_LANE_STORE = 24
H_LANE_STORE   = 0.9

N_DOCK     =  4

W_FLOOR    = N_DOCK * W_DOCK
H_FLOOR    = H_FRONT + H_LANE + H_MANEUVER + N_COMP_Y * H_COMPARTMENT + H_REAR


H_ROLL_CONTAINER = 0.7
W_ROLL_CONTAINER = 0.7

W1_ROBOT = 0.3
W2_ROBOT = 0.9
H1_ROBOT = 0.7
H2_ROBOT = 0.4

class BufferLane:
    def __init__(self, dock, lane):
        if dock<0 or N_DOCK<=dock: return
        if lane<0 or N_LANE<=lane: return

        self.dock = dock
        self.lane = lane

        s = (W_DOCK - W_DOWN - W_UP - W_LANE) / (N_LANE - 1)
        self.w1 = W_DOWN + dock * W_DOCK + lane * s
        self.w2 = self.w1 + W_LANE
        self.h1 = H_FRONT
        self.h2 = self.h1 + H_LANE
        if lane>=N_LANE/2:
            self.h1 += H_LANE
            self.h2 -= H_LANE

        self.store = []
        mid  = (self.w1 + self.w2) / 2
        if lane>=N_LANE / 2:
            step = (self.h2 - self.h1 + H_LANE_STORE) / (MAX_LANE_STORE - 1)
            self.store_coord_dict = dict([(r, (mid, self.h1-H_LANE_STORE/2 + r*step)) for r in range(MAX_LANE_STORE)])
        else:
            step = (self.h2 - self.h1 - H_LANE_STORE) / (MAX_LANE_STORE - 1)
            self.store_coord_dict = dict([(r, (mid, self.h1+H_LANE_STORE/2 + r*step)) for r in range(MAX_LANE_STORE)])

    def get_grid_coords(self):
        if self.lane>=N_LANE / 2: return self.store_coord_dict[0]                # top store
        else:                     return self.store_coord_dict[MAX_LANE_STORE-1] # top store

    def draw(self, floor_plan):
        # draw box
        pt1 = floor_plan.pnt_from_coords(self.w1, self.h1)
        pt2 = floor_plan.pnt_from_coords(self.w2, self.h2)
        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, WHITE, -1)
        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, BLACK, 1)

        # draw arrows
        w   = (self.w1 + self.w2) / 2
        pt1 = floor_plan.pnt_from_coords(w, 0.8*self.h1 + 0.2*self.h2)
        pt2 = floor_plan.pnt_from_coords(w, 0.2*self.h1 + 0.8*self.h2)
        floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt1, pt2, (100, 0, 0), 3)

        for rol in self.store:
            rol.draw(floor_plan)

    def store_roll_container(self, rol):
        rol.w, rol.h = self.store_coord_dict[MAX_LANE_STORE-1-len(self.store)]
        rol.o        = 1 if self.h2>self.h1 else 3
        self.store.append(rol)

    def pop_roll_container(self):
        if len(self.store)<=0:
            return None
        return self.store.pop()

class BufferStore:
    def __init__(self, dock, buffer):
        if dock<0   or N_DOCK<=dock: return
        if buffer<0 or N_BUFFER_STORE<=buffer: return

        self.dock   = dock
        self.buffer = buffer
        self.h1     = H_FRONT + H_LANE + H_MANEUVER
        self.h2     = self.h1 + H_BUFFER_STORE
        if N_BUFFER_STORE==1:
            self.w1  = dock*W_DOCK + (W_DOWN+W_DOCK-W_UP-W_BUFFER_STORE)/2
            self.w2  = self.w1 + W_BUFFER_STORE

            # surrounding path coords
            self.w1_ext =  dock   *W_DOCK + W_DOWN/2
            self.w2_ext = (dock+1)*W_DOCK - W_UP/2

        else:
            step     = (W_DOCK - W_DOWN-W_UP - W_BUFFER_STORE)/(N_BUFFER_STORE-1)
            self.w1  = W_DOWN + dock*W_DOCK + buffer * step
            self.w2  = self.w1 + W_BUFFER_STORE

            # surrounding path coords
            if buffer==0:                 self.w1_ext = self.w1 - (self.w1-dock*W_DOCK)/2
            else:                         self.w1_ext = self.w1 - (step-W_BUFFER_STORE)/2

            if buffer==N_BUFFER_STORE-1:  self.w2_ext = self.w2 + ((dock+1)*W_DOCK-self.w2)/2
            else:                         self.w2_ext = self.w2 + (step-W_BUFFER_STORE)/2

        self.h1_ext = self.h1 - H_LEFT / 2
        self.h2_ext = self.h2 + (H_FLOOR-self.h2) / 2

    def get_grid_coords(self, row=-1, col=-1, corner=-1):
        if row<0 or col<0:
            if corner==0: return self.w1_ext, self.h1_ext  # lower left
            if corner==1: return self.w2_ext, self.h1_ext  # lower right
            if corner==2: return self.w2_ext, self.h2_ext  # upper right
            if corner==3: return self.w1_ext, self.h2_ext  # upper left

        else:
            w = self.w1 + (col + 0.5) * W_COMPARTMENT
            h = self.h1 + (row + 0.5) * H_COMPARTMENT
            return w, h

    def get_lowest_row_coords(self, left=True):
        if left: return self.w1_ext, self.h1 + 0.5 * H_COMPARTMENT
        else:    return self.w2_ext, self.h1 + 0.5 * H_COMPARTMENT

    def draw(self, floor_plan):
        for i in range(N_COMP_X):
            w1 = self.w1 + i * W_COMPARTMENT
            w2 = self.w1 + (i + 1) * W_COMPARTMENT
            for j in range(N_COMP_Y):
                h1 = self.h1 + (j + 1) * H_COMPARTMENT
                h2 = self.h1 + j * H_COMPARTMENT

                pt1 = floor_plan.pnt_from_coords(w1, h1)
                pt2 = floor_plan.pnt_from_coords(w2, h2)

                floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, BLACK, 1)

    def draw_circulation(self, floor_plan):
        # horizontal, to left
        pt1 = floor_plan.pnt_from_coords(self.w1, self.h2_ext)
        pt2 = floor_plan.pnt_from_coords(self.w2, self.h2_ext)
        floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt2, pt1, (0, 200, 0), 3)

        # vertical, downwards
        pt1 = floor_plan.pnt_from_coords(self.w1_ext, self.h1)
        pt2 = floor_plan.pnt_from_coords(self.w1_ext, self.h2)
        floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt2, pt1, (0, 200, 0), 3)

        # vertical, upwards
        if self.buffer==N_BUFFER_STORE-1:
            pt1 = floor_plan.pnt_from_coords(self.w2_ext, self.h1)
            pt2 = floor_plan.pnt_from_coords(self.w2_ext, self.h2)
            floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt1, pt2, (0, 200, 0), 3)

class Parking:
    def __init__(self, dock):
        if dock<0 or N_DOCK<=dock: return

        self.dock = dock
        self.w1   = dock * W_DOCK + W_DOWN
        self.w2   = self.w1 + W_DOCK - W_UP - W_DOWN
        self.h1   = H_FRONT + H_LANE + H_RIGHT
        self.h2   = self.h1 + H_PARK

        # surrounding circulation box
        self.w1_ext = self.w1 - W_DOWN/2
        self.w2_ext = self.w2 + W_UP/2
        self.h1_ext = self.h1 - H_RIGHT/2
        self.h2_ext = self.h2 + H_LEFT/2

        self.n_parc_spot = int((self.w2-self.w1)/W_PARK_PLACE)
        h                = (self.h2+self.h1)/2
        step             = (self.w2-self.w1-W_PARK_PLACE)/(self.n_parc_spot-1)
        self.coord_dict  = dict([(p, (self.w1 + W_PARK_PLACE / 2 + p * step, h)) for p in range(self.n_parc_spot)])

    def get_grid_coords(self, park=-1, corner=-1):
        if park<0:
            if corner==0: return (self.w1_ext, self.h1_ext)  # lower left
            if corner==1: return (self.w2_ext, self.h1_ext)  # lower right
            if corner==2: return (self.w2_ext, self.h2_ext)  # upper right
            if corner==3: return (self.w1_ext, self.h2_ext)  # upper left

        return self.coord_dict[park]

    def draw(self, floor_plan):
        pt1 = floor_plan.pnt_from_coords(self.w1, self.h1)
        pt2 = floor_plan.pnt_from_coords(self.w2, self.h2)

        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, (0, 0, 100), -1)
        for p in self.coord_dict:
            pnt               = floor_plan.pnt_from_coords(*self.coord_dict[p])
            floor_plan.figure = cv.circle(floor_plan.figure, pnt, 5, (0,0,180), -1)

        text = f"parking {self.dock:d}"
        pnt  = floor_plan.pnt_from_coords(self.w1 + 0.2 * (self.w2-self.w1), self.h1 + 0.3 * (self.h2 - self.h1))
        font = cv.FONT_HERSHEY_SIMPLEX
        floor_plan.figure = cv.putText(floor_plan.figure, text, pnt, font, 0.7, WHITE)

    def draw_circulation(self, floor_plan):
        pt1 = floor_plan.pnt_from_coords(self.w1, self.h1_ext)
        pt2 = floor_plan.pnt_from_coords(self.w2, self.h1_ext)
        floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt1, pt2, (0, 200, 0), 3)

        # horizontal, to left
        pt1 = floor_plan.pnt_from_coords(self.w1, self.h2_ext)
        pt2 = floor_plan.pnt_from_coords(self.w2, self.h2_ext)
        floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt2, pt1, (0, 200, 0), 3)

        # vertical, downwards
        pt1 = floor_plan.pnt_from_coords(self.w1_ext, self.h1)
        pt2 = floor_plan.pnt_from_coords(self.w1_ext, self.h2)
        floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt2, pt1, (0, 200, 0), 3)

        # vertical, upwards
        pt1 = floor_plan.pnt_from_coords(self.w2_ext, self.h1)
        pt2 = floor_plan.pnt_from_coords(self.w2_ext, self.h2)
        floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt1, pt2, (0, 200, 0), 3)

class Dock:
    def __init__(self, dock):
        if dock<0 or N_DOCK<=dock: return

        self.dock = dock
        self.w1   = (dock + 0.2) * W_DOCK
        self.w2   = (dock + 0.8) * W_DOCK
        self.h1   = 0.
        self.h2   = -H_LANE / 8

    def draw(self, floor_plan):
        pt1 = floor_plan.pnt_from_coords(self.w1, self.h1)
        pt2 = floor_plan.pnt_from_coords(self.w2, self.h2)

        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, (100, 0, 0), -1)

        text = f"dock {self.dock:d}"
        pnt = floor_plan.pnt_from_coords(self.w1 + 0.1 * W_DOCK, 0.8 * self.h2)
        font = cv.FONT_HERSHEY_SIMPLEX
        floor_plan.figure = cv.putText(floor_plan.figure, text, pnt, font, 0.7, WHITE)

        if self.dock>0:
            floor_plan.figure = cv.line(floor_plan.figure,
                                        floor_plan.pnt_from_coords(self.dock * W_DOCK, 0.),
                                        floor_plan.pnt_from_coords(self.dock * W_DOCK, H_FLOOR), BLACK, 1)

class FloorPlan:
    def __init__(self):
        border_w          = 70
        border_h          = 100
        self.fig_width    = 1000
        self.fig_height   = border_h + int((self.fig_width-border_h) * H_FLOOR / W_FLOOR)
        self.top_left     = (20, 50)
        self.bottom_right = (self.fig_width-(border_w-self.top_left[0]), self.fig_height-(border_h-self.top_left[1]))

        self.figure       = np.full((self.fig_height, self.fig_width, 3), 255, np.uint8)
        self.figure       = cv.rectangle(self.figure, self.top_left, self.bottom_right, BLACK, 2)

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

    def draw(self):
        for dock in range(N_DOCK):
            for store in range(N_BUFFER_STORE):
                self.buffer_stores[(dock, store)].draw(self)

            self.docks[dock].draw(self)
            self.parkings[dock].draw(self)
            for lane in range(N_LANE):
                self.buffer_lanes[(dock, lane)].draw(self)

            self.__draw_circulation(dock)

        self.draw_grid()
        self.__draw_legends()

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
            if buffer_lane>N_BUFFER_STORE: return
            return self.buffer_lanes[(dock, buffer_lane)].get_grid_coords()

        if parking>=0:
            if parking>=self.parkings[dock].n_parc_spot: return
            return self.parkings[dock].get_grid_coords(park=parking)

        if store>=0:
            if store>=N_BUFFER_STORE: return
            return self.buffer_stores[(dock, store)].get_grid_coords(row=row, col=col)

    def get_shortest_path(self, dock1, dock2, buffer_lane1=-1, parking1=-1, store1=-1, row1=-1, buffer_lane2=-1, parking2=-1, store2=-1, row2=-1):
        if dock1<0 or N_DOCK<=dock1: return
        if dock2<0 or N_DOCK<=dock2: return

        coords1 = self.get_item_coords(dock1, buffer_lane1, parking1, store1, row1, 0)
        coords2 = self.get_item_coords(dock2, buffer_lane2, parking2, store2, row2, 0)

        if not coords1 is None and not coords2 is None: return self.grid_graph.get_shortest_path(coords1, coords2)

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

class RollContainer:
    def __init__(self, w, h, orientation, color):
        self.w   = w
        self.h   = h
        self.o   = orientation
        self.col = color

    def draw(self, floor_plan):
        w1 = self.w - 0.5*W_ROLL_CONTAINER
        w2 = w1 + W_ROLL_CONTAINER
        h1 = self.h - 0.5*H_ROLL_CONTAINER
        h2 = h1 + H_ROLL_CONTAINER

        pt1 = floor_plan.pnt_from_coords(w1, h1)
        pt2 = floor_plan.pnt_from_coords(w2, h2)

        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, self.col, -1)
        f = 0.8
        if self.o==0: # left
            pt = floor_plan.pnt_from_coords(f*w2+(1-f)*w1, (h1+h2)/2)
        elif self.o==1: # up
            pt = floor_plan.pnt_from_coords((w1+w2)/2, f*h2+(1-f)*h1)
        elif self.o==2: # right
            pt = floor_plan.pnt_from_coords(f*w1+(1-f)*w2, (h1+h2)/2)
        elif self.o==3: # down
            pt = floor_plan.pnt_from_coords((w1+w2)/2, f*h1+(1-f)*h2)
        else:
            return

        floor_plan.figure = cv.circle(floor_plan.figure, pt, 5, (100, 100, 100), -1)

    def rotate(self, left):
        step   = 1 if left else -1
        self.o = (self.o + step)%4

class Robot:
    def __init__(self, w, h, orientation, color):
        self.w   = w
        self.h   = h
        self.o   = orientation
        self.rot = [[1, 0], [0, 1]]
        self.col = color
        self.rol = None

        self.__update_rotmat()

    def __update_rotmat(self):
        if self.o==0:
            self.rot = [[1, 0], [0, 1]]
        elif self.o==1:
            self.rot = [[0, -1], [1, 0]]
        elif self.o==2:
            self.rot = [[-1, 0], [0, -1]]
        elif self.o==3:
            self.rot = [[0, 1], [-1, 0]]

    def __trans_coords(self, w, h):
        """
        return rotated coordinates of point (w,h), with (self.w, self.h) as centre
        """
        x  = w-self.w
        y  = h-self.h
        xa = self.rot[0][0]*x + self.rot[0][1]*y
        ya = self.rot[1][0]*x + self.rot[1][1]*y
        return xa+self.w, ya+self.h

    def draw(self, floor_plan):
        # Engine
        w1  = self.w - 0.5*(W1_ROBOT+W2_ROBOT)
        w2  = w1 + W1_ROBOT
        h1  = self.h - 0.5*H1_ROBOT
        h2  = h1 + H1_ROBOT
        pt1 = floor_plan.pnt_from_coords(*self.__trans_coords(w1, h1))
        pt2 = floor_plan.pnt_from_coords(*self.__trans_coords(w2, h2))

        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, self.col, -1)

        # carrier platform
        w1  = self.w + 0.5*(W1_ROBOT-W2_ROBOT)
        w2  = w1 + W2_ROBOT
        h1  = self.h - 0.5*H2_ROBOT
        h2  = h1 + H2_ROBOT
        pt1 = floor_plan.pnt_from_coords(*self.__trans_coords(w1, h1))
        pt2 = floor_plan.pnt_from_coords(*self.__trans_coords(w2, h2))

        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, self.col, -1)

        if not self.rol is None:
            self.rol.draw(floor_plan)

    def move(self, step):
        if self.o==0:
            self.w += step
        elif self.o==1:
            self.h += step
        elif self.o==2:
            self.w -= step
        elif self.o==3:
            self.h -= step

        self.w = max(0., min(W_FLOOR, self.w))
        self.h = max(0., min(H_FLOOR, self.h))
        if not self.rol is None:
            self.rol.w = self.w
            self.rol.h = self.h

    def rotate(self, left):
        step = 1 if left else -1
        self.o = (self.o + step) % 4
        self.__update_rotmat()

        if not self.rol is None:
            self.rol.rotate(left)

    def load_roll_container(self, rol):
        self.rol   = rol
        self.rol.w = self.w
        self.rol.h = self.h
        self.rol.o = self.o

    def unload_roll_container(self):
        self.rol   = None

import random
def main():
    fp = FloorPlan()
    fp.draw()
    fp.imshow("Test")
    cv.waitKey(0)

    n_park = fp.parkings[0].n_parc_spot
    robots = [Robot(*fp.get_item_coords(dock=0, parking=p), 3, (200, 0, 0)) for p in range(n_park)]

    for k in range(10):
        rr = random.randint(0,255)
        gg = random.randint(0,255)
        bb = random.randint(0,255)
        rol = RollContainer(W_FLOOR / 2, H_FLOOR / 2, 3, (rr, gg, bb))

        dd = random.randint(0, N_DOCK-1)
        ll = random.randint(0, N_LANE-1)

        fp.buffer_lanes[(0, 1)].store_roll_container(rol)
        fp.draw()
        for rob in robots:
            rob.draw(fp)
            fp.imshow("Test")

        fp.imshow("Test")
        cv.waitKey(500)


    cv.waitKey(0)
    cv.destroyAllWindows()

    return


    rol = RollContainer(W_FLOOR/2, H_FLOOR/2, 3, (200, 2000, 50))
    rob = Robot(W_FLOOR/2, H_FLOOR/2, 3, (200, 0, 0))
    for k in range(20):
        n_step = random.randint(0, 100)
        for s in range(n_step):
            rob.move(0.05)
            fp = FloorPlan()
            rob.draw(fp)
            if rob.rol is None:
                rol.draw(fp)
            fp.imshow("Test")
            cv.waitKey(1)

        rot_left = (1==random.randint(0,1))
        rob.rotate(rot_left)
        if k==4:
            rob.load_roll_container(rol)
        if k==14:
            rob.unload_roll_container()
        cv.waitKey(500)

    cv.waitKey(0)
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()