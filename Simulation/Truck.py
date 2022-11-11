import cv2 as cv
import numpy as np

from XdockParams import MAX_TRUCK_LOAD

class Truck:
    def __init__(self, t_arrive, t_departure, color, destination=None, prios=None, roll_containers=None):
        self.color       = color
        self.arrival     = t_arrive
        self.departure   = t_departure

        self.dock        = -1

        self.destination = destination
        self.prios       = prios
        self.truck_load  = [] if roll_containers is None else [rol for rol in roll_containers]

    def __str__(self):
        text  = f"arrival = {self.arrival:7.2f}\n"
        text += f"departure = {self.departure:7.2f}\n"
        text += f"dock = {self.dock:d}\n"
        if self.destination:
            text +=  f"destination = {self.destination:s}\n"
        if self.prios:
            text +=  f"prios = {str(self.prios):s}\n"
        text += f"nrc = {len(self.truck_load):d}\n"
        return text

    def draw(self, floor_plan, dock):
        s = 0.2*(dock.w2-dock.w1)
        x = 0.6*dock.w1 + 0.4*dock.w2
        y = 1.1*dock.h1 - 0.1*dock.h2

        truck = [(-1.5, 0.0), (2.5, 0.0), (2.5, 0.85), (1.5, 0.85), (1.5, 1.7), (-1.5, 1.7)]
        truck = [(x + qx * s, y + (qy + 0.5) * s) for (qx, qy) in truck]
        truck = [list(floor_plan.pnt_from_coords(*q)) for q in truck]
        floor_plan.figure = cv.polylines(floor_plan.figure, [np.array(truck)], True, self.color, 1)

        if len(self.truck_load)>0:
            w    = 3*len(self.truck_load)/MAX_TRUCK_LOAD
            load = [(1.5-w,0.0), (1.5,1.7)]
            load = [(x + qx * s, y + (qy + 0.5) * s) for (qx, qy) in load]
            load = [list(floor_plan.pnt_from_coords(*q)) for q in load]
            cv.rectangle(floor_plan.figure, load[0], load[1], self.color, -1)

        wheels = [(-1.0, 0.0), (2.0, 0.0)]
        wheels = [(x + qx*s, y + (qy + 0.5) * s) for (qx, qy) in wheels]
        wheels = [floor_plan.pnt_from_coords(*q) for q in wheels]
        for q in wheels:
            floor_plan.figure = cv.circle(floor_plan.figure, q, 5, self.color, -1)


def main():
    truck_list = trucks_from_file()
    print(truck_list)

if __name__=="__main__":
    main()