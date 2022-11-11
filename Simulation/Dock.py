import cv2 as cv

from XdockParams import N_DOCK, W_DOCK, H_LANE,  \
                        WHITE, MAX_TRUCK_LOAD,\
                        round_coords


class Dock:
    def __init__(self, dock):
        if dock<0 or N_DOCK<=dock: return

        self.dock = dock
        self.name = f"dock {self.dock:d}"
        self.w1   = (dock + 0.05) * W_DOCK
        self.w2   = (dock + 0.95) * W_DOCK
        self.h1   = -H_LANE / 8
        self.h2   = 0.

        self.w1, self.h1 = round_coords((self.w1, self.h1))
        self.w2, self.h2 = round_coords((self.w2, self.h2))

        self.truck_list = []
        self.truck_in   = None
        self.truck_out  = None
        self.color      = (200,0,0)

    def set_truck_list(self, truck_list):
        self.truck_list = sorted([truck for truck in truck_list if truck.dock==self.dock], key=lambda t: t.arrival)

    def set_color(self, col):
        self.color = col

    def set_name(self, name):
        self.name = name

    def undock_truck(self, time_sec):
        if self.truck_in:
            if self.truck_in.departure<time_sec:
                self.truck_in = None
        elif self.truck_out:
            if self.truck_out.departure<time_sec:
                self.truck_out = None

    def start_unloading(self, truck):
        self.truck_out = None
        self.truck_in  = truck

    def start_loading(self, truck):
        self.truck_in  = None
        self.truck_out = truck

    def can_be_unloaded(self):
        return not self.truck_in is None and len(self.truck_in.truck_load)>0

    def can_be_loaded(self):
        return not self.truck_out is None and MAX_TRUCK_LOAD-len(self.truck_out.truck_load)>0

    def unload_next_roll_container(self):
        return self.truck_in.truck_load.pop(0)

    def load_next_roll_container(self, rol):
        if self.truck_out and len(self.truck_out.truck_load)<MAX_TRUCK_LOAD:
            self.truck_out.truck_load.append(rol)

    def draw(self, floor_plan):
        if self.truck_in is None and self.truck_out is None:
            pt1 = floor_plan.pnt_from_coords(self.w1, self.h2-0.15)
            pt2 = floor_plan.pnt_from_coords(self.w2, self.h1)

            floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, self.color, -1)

            pnt  = floor_plan.pnt_from_coords(self.w1 + 0.05 * W_DOCK, 0.8 * self.h1)
            font = cv.FONT_HERSHEY_SIMPLEX
            floor_plan.figure = cv.putText(floor_plan.figure, self.name, pnt, font, 0.4, WHITE)

        else:
            if self.truck_in:
                w                 = self.w1
                pt1               = floor_plan.pnt_from_coords(w, 0.9 * self.h1 + 0.1 * self.h2)
                pt2               = floor_plan.pnt_from_coords(w, 0.1 * self.h1 + 0.9 * self.h2)
                floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt1, pt2, (200, 200, 200), 2, tipLength=0.3)
                self.truck_in.draw(floor_plan, self)
            elif self.truck_out:
                w                 = self.w2
                pt1               = floor_plan.pnt_from_coords(w, 0.1 * self.h1 + 0.9 * self.h2)
                pt2               = floor_plan.pnt_from_coords(w, 0.9 * self.h1 + 0.1 * self.h2)
                floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt1, pt2, (200, 200, 200), 2, tipLength=0.3)
                self.truck_out.draw(floor_plan, self)

