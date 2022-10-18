import cv2 as cv
from FloorPlan import round_coords, \
                      N_DOCK, W_DOCK, H_FRONT, \
                      W_LANE, H_LANE, N_LANE, H_LANE_STORE, MAX_LANE_STORE, BUFFER_LANE_SPEED, \
                      TIME_STEP_S, \
                      BLACK, WHITE
class BufferLane:
    def __init__(self, dock, lane):
        if dock<0 or N_DOCK<=dock: return
        if lane<0 or N_LANE<=lane: return

        self.lane_up = lane<N_LANE/2
        self.dock    = dock
        self.lane    = lane

        self.w1 = (W_DOCK/N_LANE-W_LANE)/2 + self.lane*W_DOCK/N_LANE + dock * W_DOCK
        self.w2 = self.w1 + W_LANE
        self.h1 = H_FRONT
        self.h2 = self.h1 + H_LANE
        if not self.lane_up:
            self.h1 += H_LANE
            self.h2 -= H_LANE

        self.store = []
        mid  = (self.w1 + self.w2) / 2
        if self.lane_up:
            step = (self.h2 - self.h1 - H_LANE_STORE) / (MAX_LANE_STORE - 1)
            self.store_coord_dict = dict([(r, round_coords((mid, self.h1+H_LANE_STORE/2 + r*step))) for r in range(MAX_LANE_STORE)])
        else:
            step = (self.h2 - self.h1 + H_LANE_STORE) / (MAX_LANE_STORE - 1)
            self.store_coord_dict = dict([(r, round_coords((mid, self.h1-H_LANE_STORE/2 + r*step))) for r in range(MAX_LANE_STORE)])

        self.w1, self.h1 = round_coords((self.w1, self.h1))
        self.w2, self.h2 = round_coords((self.w2, self.h2))

    def time_step(self):
        move = BUFFER_LANE_SPEED * TIME_STEP_S
        if self.lane_up:
            for r, rol in enumerate(self.store):
                rol.h = min(rol.h+move, self.store_coord_dict[MAX_LANE_STORE-1-r][1])
        else:
            for r, rol in enumerate(self.store):
                rol.h = max(rol.h-move, self.store_coord_dict[MAX_LANE_STORE-1-r][1])

    def get_grid_coords(self):
        # return top store
        if self.lane_up: return self.store_coord_dict[MAX_LANE_STORE-1]
        else:            return self.store_coord_dict[0]

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
        if len(self.store)>=MAX_LANE_STORE: return

        rol.w, rol.h = self.store_coord_dict[0]
        rol.o = 1 if self.lane_up else 3
        self.store.append(rol)

    def pop_roll_container(self):
        if len(self.store)>0:
            return self.store.pop(0)