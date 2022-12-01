import cv2 as cv
from XdockParams import W_DOCK, H_LEFT, H_RIGHT, H_FRONT, H_PARK, W_PARK_PLACE, \
                        round_coords
from ModelParameters import ModelParams as M

class Parking:
    def __init__(self, dock):
        if dock<0 or M.N_DOCK<=dock: return

        self.dock = dock
        self.w1   = dock * W_DOCK + M.W_DOWN
        self.w2   = self.w1 + W_DOCK - M.W_UP - M.W_DOWN
        self.h1   = H_FRONT + M.H_LANE + H_RIGHT
        self.h2   = self.h1 + H_PARK

        # surrounding circulation box
        self.w1_ext = self.w1 - M.W_DOWN/2
        self.w2_ext = self.w2 + M.W_UP/2
        self.h1_ext = self.h1 - H_RIGHT/2
        self.h2_ext = self.h2 + H_LEFT/2

        self.w1, self.h1         = round_coords((self.w1, self.h1))
        self.w2, self.h2         = round_coords((self.w2, self.h2))
        self.w1_ext, self.h1_ext = round_coords((self.w1_ext, self.h1_ext))
        self.w2_ext, self.h2_ext = round_coords((self.w2_ext, self.h2_ext))

        self.n_parc_spot   = int(0.5+(self.w2-self.w1)/W_PARK_PLACE)
        h                  = (self.h2+self.h1)/2
        step               = (self.w2-self.w1-W_PARK_PLACE)/(self.n_parc_spot-1)
        self.coord_dict    = dict([(p, round_coords((self.w1+W_PARK_PLACE/2 + p*step, h))) for p in range(self.n_parc_spot)])

    def get_grid_coords(self, park=-1, corner=-1):
        if park<0:
            if corner==0: return self.w1_ext, self.h1_ext  # lower left
            if corner==1: return self.w2_ext, self.h1_ext  # lower right
            if corner==2: return self.w2_ext, self.h2_ext  # upper right
            if corner==3: return self.w1_ext, self.h2_ext  # upper left

        return self.coord_dict[park]

    def get_park_positions(self):
        return [self.get_grid_coords(park) for park in range(self.n_parc_spot)]

    def draw(self, floor_plan):
        pt1 = floor_plan.pnt_from_coords(self.w1, self.h1)
        pt2 = floor_plan.pnt_from_coords(self.w2, self.h2)

        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, (0, 0, 100), -1)
        for p in self.coord_dict:
            pnt               = floor_plan.pnt_from_coords(*self.coord_dict[p])
            floor_plan.figure = cv.circle(floor_plan.figure, pnt, 5, (0,0,180), -1)

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