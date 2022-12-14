import cv2 as cv
import numpy as np

from XdockParams import MAX_TRUCK_LOAD, TIME_DOCK_INBOUND, TIME_DOCK_OUTBOUND, TIME_LOAD_RC_TRUCK, TIME_UNLOAD_RC_TRUCK, TIME_STEP_S,\
                        BLACK
from ModelParameters import get_log_filename

from functools import update_wrapper, partial
class TruckLogger:
    def __init__(self, func):
        update_wrapper(self, func)
        self.func      = func
        self.log_file  = ""

    def __get__(self, obj, objtype):
        return partial(self.__call__, obj)

    def __call__(self, truck, *args, **kwargs):
        result = self.func(truck, *args, **kwargs)

        if self.log_file=="":
            self.log_file = get_log_filename("Trucks")
            dummy = Truck(0., 0., (0, 0, 0))
            with open(self.log_file, "w") as fp:
                fp.write(dummy._get_log_line(header=True) + '\n')

        with open(self.log_file, "a") as fp:
            fp.write(truck._get_log_line() + '\n')

        return result

class Truck:
    lastID = 0
    def __init__(self, t_arrive, t_departure, color, destination=None, prios=None, roll_containers=None, ID = None):
        self.color       = color
        self.arrival     = t_arrive
        self.departure   = t_departure

        self.dock        = -1

        self.destination = destination
        self.inbound     = destination is None
        self.prios       = prios
        self.truck_load  = [] if roll_containers is None else [rol for rol in roll_containers]

        if ID is None:
            self.ID         = str(Truck.lastID)
            Truck.lastID   += 1
        else:
            self.ID         = str(ID)

        self.__docked       = False
        self.__dock_time    = -1.
        self.__dead_time_rc = 0.

    def __str__(self):
        text  = f"ID = {str(self.ID):s}\n"
        text += f"docked = {str(self.__docked):s}\n"
        text += f"arrival = {self.arrival/3600:7.2f}\n"
        text += f"departure = {self.departure/3600:7.2f}\n"
        text += f"inbound = {str(self.inbound):s}\n"
        text += f"dock = {self.dock:d}\n"
        text += f"destination = {str(self.destination):s}\n"
        text += f"prios = {str(self.prios):s}\n"
        text += f"nrc = {len(self.truck_load):d}\n"
        return text

    def _get_log_line(self, header=False):
        h = 0 if header else 1
        return "\t".join(kw.split('=')[h].strip() for kw in str(self).split("\n") if len(kw)>1)

    def draw(self, floor_plan, dock):
        s = 0.2*(dock.w2-dock.w1)
        x = 0.6*dock.w1 + 0.4*dock.w2
        y = 1.1*dock.h1 - 0.1*dock.h2

        truck = [(-1.5, 0.0), (2.5, 0.0), (2.5, 0.85), (1.5, 0.85), (1.5, 1.7), (-1.5, 1.7)]
        if self.inbound:
            truck = [(-w+1,h) for (w,h) in truck]
        truck = [(x + qx * s, y + (qy + 0.5) * s) for (qx, qy) in truck]
        truck = [list(floor_plan.pnt_from_coords(*q)) for q in truck]
        floor_plan.figure = cv.polylines(floor_plan.figure, [np.array(truck)], True, self.color, 1)

        if len(self.truck_load)>0:
            w    = 3*len(self.truck_load)/MAX_TRUCK_LOAD
            load = [(w-0.5,0.0), (-0.5,1.7)] if self.inbound else [(-1.5+w,0.0), (-1.5,1.7)]
            load = [(x + qx * s, y + (qy + 0.5) * s) for (qx, qy) in load]
            load = [list(floor_plan.pnt_from_coords(*q)) for q in load]
            cv.rectangle(floor_plan.figure, load[0], load[1], self.color, -1)

            pnt  = floor_plan.pnt_from_coords(x, 1.4*dock.h1 - 0.4*dock.h2)
            font = cv.FONT_HERSHEY_SIMPLEX
            nrc  = len(self.truck_load)
            floor_plan.figure = cv.putText(floor_plan.figure, str(nrc), pnt, font, 0.5, BLACK)

        wheels = [(2.0, 0.0), (-1.0, 0.0)] if self.inbound else [(-1.0, 0.0), (2.0, 0.0)]
        wheels = [(x + qx*s, y + (qy + 0.5) * s) for (qx, qy) in wheels]
        wheels = [floor_plan.pnt_from_coords(*q) for q in wheels]
        for q in wheels:
            floor_plan.figure = cv.circle(floor_plan.figure, q, 5, self.color, -1)

    def time_step(self):
        if not self.__docked: return

        self.__dock_time += TIME_STEP_S
        if self.__dock_time>=TIME_DOCK_INBOUND:
            self.__dead_time_rc += TIME_STEP_S

    @TruckLogger
    def start_docking(self):
        self.__docked    = True
        self.__dock_time = 0.
        if self.inbound:
            self.__dead_time_rc = -TIME_UNLOAD_RC_TRUCK
        else:
            self.__dead_time_rc = -TIME_LOAD_RC_TRUCK

    def can_be_undocked(self):
        if not self.__docked: return False
        return self.__dock_time>=self.departure-self.arrival

    @TruckLogger
    def undock(self):
        self.__docked = False

    def can_be_unloaded(self):
        if not self.__docked or not self.inbound: return False
        if self.__dock_time<self.__dock_time<TIME_DOCK_INBOUND: return False
        if self.departure-self.arrival-TIME_DOCK_OUTBOUND< self.__dock_time: return False
        if self.__dead_time_rc<=0.: return False
        return len(self.truck_load)>0

    def can_be_loaded(self):
        if not self.__docked or self.inbound: return False
        if self.__dock_time<self.__dock_time<TIME_DOCK_INBOUND: return False
        if self.departure-self.arrival-TIME_DOCK_OUTBOUND< self.__dock_time: return False
        if self.__dead_time_rc<=0.: return False
        return MAX_TRUCK_LOAD-len(self.truck_load)>0

    def unload_next_roll_container(self):
        self.__dead_time_rc -= TIME_UNLOAD_RC_TRUCK
        return self.truck_load.pop(0)

    def load_next_roll_container(self, rol):
        self.__dead_time_rc -= TIME_LOAD_RC_TRUCK
        if len(self.truck_load)<MAX_TRUCK_LOAD:
            self.truck_load.append(rol)