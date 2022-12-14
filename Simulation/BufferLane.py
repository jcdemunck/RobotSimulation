import cv2 as cv
from collections import namedtuple

from XdockParams import round_coords, \
                      W_DOCK, H_FRONT, \
                      W_LANE, N_LANE, H_LANE_STORE, \
                      TIME_STEP_S, \
                      BLACK, WHITE

from ModelParameters import ModelParams as M

RollContainerIO = namedtuple("rc_in_out", "eta lane roll_container")

class BufferLane:
    def __init__(self, dock, lane):
        if dock<0 or M.N_DOCK<=dock: return
        if lane<0 or   N_LANE<=lane: return

        self.lane_up = lane>=N_LANE/2
        self.dock    = dock
        self.lane    = lane

        self.w1 = (W_DOCK/N_LANE-W_LANE)/2 + self.lane*W_DOCK/N_LANE + dock * W_DOCK
        self.w2 = self.w1 + W_LANE
        self.h1 = H_FRONT
        self.h2 = self.h1 + M.H_LANE

        self.dead_time_up   = 0.
        self.dead_time_down = 0.
        self.store          = []
        mid  = (self.w1 + self.w2) / 2
        step = (self.h2 - self.h1 - H_LANE_STORE) / (M.MAX_LANE_STORE - 1)
        if self.lane_up:
            self.store_coord_dict = dict([(r, round_coords((mid, self.h1+H_LANE_STORE/2 + r*step))) for r in range(M.MAX_LANE_STORE)])
        else:
            self.store_coord_dict = dict([(r, round_coords((mid, self.h2-H_LANE_STORE/2 - r*step))) for r in range(M.MAX_LANE_STORE)])

        self.w1, self.h1 = round_coords((self.w1, self.h1))
        self.w2, self.h2 = round_coords((self.w2, self.h2))

        self.n_store_reserved = 0

    def __str__(self):
        text  = f"lane_up  = {str(self.lane_up):s} \n"
        text += f"dock     = {self.dock:d} \n"
        text += f"lane     = {self.lane:d}\n"
        text += f"n_store_reserved = {self.n_store_reserved:d}\n"
        text += f"n_stored = {len(self.store):d}\n"
        return text

    def get_expected_roll_containers(self):
        """
            return list of expected roll containers as tuples:
            (expected time of availability, lane, roll container)
        """
        return [RollContainerIO((self.store_coord_dict[M.MAX_LANE_STORE-1][1]-rol.h)/M.BUFFER_LANE_SPEED, self.lane, rol) for rol in self.store]

    def time_step(self):
        move = M.BUFFER_LANE_SPEED * TIME_STEP_S
        if self.lane_up:
            for r, rol in enumerate(self.store):
                rol.h = min(rol.h+move, self.store_coord_dict[M.MAX_LANE_STORE-1-r][1])
        else:
            for r, rol in enumerate(self.store):
                rol.h = max(rol.h-move, self.store_coord_dict[M.MAX_LANE_STORE-1-r][1])

        self.dead_time_up   += TIME_STEP_S
        self.dead_time_down += TIME_STEP_S

    def can_be_loaded(self):
        return len(self.store)<M.MAX_LANE_STORE and self.dead_time_up>=0.

    def can_be_unloaded(self):
        return len(self.store)>0 and self.store[0].h<=self.h1+H_LANE_STORE/2 and self.dead_time_down>=0.

    def get_grid_coords(self):
        # return top store
        if self.lane_up: return self.store_coord_dict[M.MAX_LANE_STORE-1]
        else:            return self.store_coord_dict[0]

    def draw(self, floor_plan):
        # draw box
        pt1 = floor_plan.pnt_from_coords(self.w1, self.h1)
        pt2 = floor_plan.pnt_from_coords(self.w2, self.h2)
        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, WHITE, -1)
        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, BLACK,  1)

        # draw arrows
        w   = (self.w1 + self.w2) / 2
        pt1 = floor_plan.pnt_from_coords(w, 0.8*self.h1 + 0.2*self.h2)
        pt2 = floor_plan.pnt_from_coords(w, 0.2*self.h1 + 0.8*self.h2)
        if self.lane_up:
            floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt1, pt2, (100, 0, 0), 3)
        else:
            floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt2, pt1, (100, 0, 0), 3)

        for rol in self.store:
            rol.draw(floor_plan)

    def get_n_stored(self):
        return len(self.store)

    def reserve_store(self):
        self.n_store_reserved += 1

    def store_roll_container(self, rol):
        if len(self.store)>=M.MAX_LANE_STORE: return

        rol.w, rol.h = self.store_coord_dict[0]
        rol.o = 1 if self.lane_up else 3
        self.store.append(rol)
        if self.lane_up:
            self.dead_time_up =-M.TIME_LOAD_BUFFER_LANE

    def pickup_roll_container(self):
        if len(self.store)>0:
            self.n_store_reserved -= 1
            if not self.lane_up:
                self.dead_time_down =-M.TIME_LOAD_BUFFER_LANE
            return self.store.pop(0)
        print("ERROR: BufferLane.pickup_roll_container(). Store empty.\n", str(self))