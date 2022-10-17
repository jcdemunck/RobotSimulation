import cv2 as cv
from FloorPlan import round_coords, \
                      N_DOCK, W_DOCK, H_LANE, H_FLOOR, \
                      BLACK, WHITE

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

    def draw(self, floor_plan):
        pt1 = floor_plan.pnt_from_coords(self.w1, self.h1)
        pt2 = floor_plan.pnt_from_coords(self.w2, self.h2)

        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, (100, 0, 0), -1)

        text = f"dock {self.dock:d}"
        pnt = floor_plan.pnt_from_coords(self.w1 + 0.05 * W_DOCK, 0.8 * self.h2)
        font = cv.FONT_HERSHEY_SIMPLEX
        floor_plan.figure = cv.putText(floor_plan.figure, text, pnt, font, 0.5, WHITE)

        if self.dock>0:
            floor_plan.figure = cv.line(floor_plan.figure,
                                        floor_plan.pnt_from_coords(self.dock * W_DOCK, 0.),
                                        floor_plan.pnt_from_coords(self.dock * W_DOCK, H_FLOOR), BLACK, 1)
