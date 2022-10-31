import cv2 as cv

from FloorPlan import round_coords, \
                      N_DOCK, W_DOCK, H_LANE,  \
                      WHITE, MAX_TRUCK_LOAD


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

        self.truck_in  = None
        self.truck_out = None
        self.color     = (200,0,0)

    def set_color(self, col):
        self.color = col

    def start_unloading(self, truck):
        self.truck_out = None
        self.truck_in  = truck

    def start_loading(self, truck):
        self.truck_in  = None
        self.truck_out = truck

    def get_nrc_input(self):
        if self.truck_in is None: return 0
        return len(self.truck_in.truck_load)

    def get_nrc_output(self):
        if self.truck_out is None: return 0
        return MAX_TRUCK_LOAD-len(self.truck_out.truck_load)

    def unload_next_roll_container(self):
        return self.truck_in.truck_load.pop(0)

    def load_next_roll_container(self, rol):
        if self.truck_out and len(self.truck_out.truck_load)<MAX_TRUCK_LOAD:
            self.truck_out.truck_load.append(rol)

    def draw(self, floor_plan):
        if self.truck_in is None and self.truck_out is None:
            pt1 = floor_plan.pnt_from_coords(self.w1, self.h1-0.15)
            pt2 = floor_plan.pnt_from_coords(self.w2, self.h2)

            floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, self.color, -1)

            text = f"dock {self.dock:d}"
            pnt  = floor_plan.pnt_from_coords(self.w1 + 0.05 * W_DOCK, 0.8 * self.h2)
            font = cv.FONT_HERSHEY_SIMPLEX
            floor_plan.figure = cv.putText(floor_plan.figure, text, pnt, font, 0.5, WHITE)

        else:
            if self.truck_in:  self.truck_in.draw(floor_plan, self)
            if self.truck_out: self.truck_out.draw(floor_plan, self)
