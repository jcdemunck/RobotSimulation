import cv2
import cv2 as cv
import numpy as np

from FloorPlan import round_coords, \
                      N_DOCK, W_DOCK, H_LANE,  \
                      WHITE



class Dock:
    def __init__(self, dock):
        if dock<0 or N_DOCK<=dock: return

        self.dock = dock
        self.w1   = (dock + 0.2) * W_DOCK
        self.w2   = (dock + 0.8) * W_DOCK
        self.h1   = 0.
        self.h2   = -H_LANE / 8

        self.w1, self.h1 = round_coords((self.w1, self.h1))
        self.w2, self.h2 = round_coords((self.w2, self.h2))

        self.rc_loading    = []
        self.rc_unloading  = []
        self.sample        = 0
        self.sample_load   = 0
        self.sample_unload = 0

        self.color         = (200,0,0)

    def set_color(self, col):
        self.color = col

    def start_unloading(self, roll_containers):
        self.rc_unloading  += roll_containers
        self.sample_unload  = self.sample

    def time_step(self):
        self.sample +=1

    def draw(self, floor_plan):
        if self.sample_load==self.sample_unload:
            pt1 = floor_plan.pnt_from_coords(self.w1, self.h1-0.15)
            pt2 = floor_plan.pnt_from_coords(self.w2, self.h2)

            floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, self.color, -1)

            text = f"dock {self.dock:d}"
            pnt  = floor_plan.pnt_from_coords(self.w1 + 0.05 * W_DOCK, 0.8 * self.h2)
            font = cv.FONT_HERSHEY_SIMPLEX
            floor_plan.figure = cv.putText(floor_plan.figure, text, pnt, font, 0.5, WHITE)

        else:
            self.__draw_truck(floor_plan, len(self.rc_loading)>0 or len(self.rc_unloading)>0)

    def __draw_truck(self, floor_plan, full):
        s = 0.2*(self.w2-self.w1)
        x = self.w1 + 0.5*(self.w2-self.w1)
        y = self.h2

        truck = [(-1.5, 0.0), (2.5, 0.0), (2.5, 1.0), (1.5, 1.0), (1.5, 2.0), (-1.5, 2.0)]
        truck = [(x + qx * s, y + (qy + 0.5) * s) for (qx, qy) in truck]
        truck = [list(floor_plan.pnt_from_coords(*q)) for q in truck]

        if full:
            floor_plan.figure = cv2.fillPoly(floor_plan.figure, [np.array(truck)], self.color)
        else:
            floor_plan.figure = cv2.polylines(floor_plan.figure, [np.array(truck)], True, self.color, 2)

        wheels = [(-1.0, 0.0), (2.0, 0.0)]
        wheels = [(x + qx*s, y + (qy + 0.5) * s) for (qx, qy) in wheels]
        wheels = [floor_plan.pnt_from_coords(*q) for q in wheels]
        for q in wheels:
            floor_plan.figure = cv2.circle(floor_plan.figure, q, 5, self.color, -1)

