import cv2 as cv
import numpy as np

from FloorPlan import MAX_TRUCK_LOAD

class Truck:
    def __init__(self, roll_containers, destination, color, t_arrive, t_departure):
        self.dock        = -1
        self.truck_load  = [] if roll_containers is None else [rol for rol in roll_containers]
        self.destination = destination
        self.color       = color
        self.arrival     = t_arrive
        self.departure   = t_departure

    def draw(self, floor_plan, dock):
        s = 0.2*(dock.w2-dock.w1)
        x = dock.w1 + 0.5*(dock.w2-dock.w1)
        y = dock.h2

        truck = [(-1.5, 0.0), (2.5, 0.0), (2.5, 1.0), (1.5, 1.0), (1.5, 2.0), (-1.5, 2.0)]
        truck = [(x + qx * s, y + (qy + 0.5) * s) for (qx, qy) in truck]
        truck = [list(floor_plan.pnt_from_coords(*q)) for q in truck]
        floor_plan.figure = cv.polylines(floor_plan.figure, [np.array(truck)], True, self.color, 2)

        if len(self.truck_load)>0:
            w    = 3*len(self.truck_load)/MAX_TRUCK_LOAD
            load = [(1.5-w,0.0), (1.5,2.0)]
            load = [(x + qx * s, y + (qy + 0.5) * s) for (qx, qy) in load]
            load = [list(floor_plan.pnt_from_coords(*q)) for q in load]
            cv.rectangle(floor_plan.figure, load[0], load[1], self.color, -1)

        wheels = [(-1.0, 0.0), (2.0, 0.0)]
        wheels = [(x + qx*s, y + (qy + 0.5) * s) for (qx, qy) in wheels]
        wheels = [floor_plan.pnt_from_coords(*q) for q in wheels]
        for q in wheels:
            floor_plan.figure = cv.circle(floor_plan.figure, q, 5, self.color, -1)
