import cv2 as cv

from XdockParams import N_DOCK, W_DOCK, H_LANE,  \
                        WHITE, TIME_LOAD_BUFFER_LANE, TIME_STEP_S,\
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
        self.truck      = None
        self.color      = (200,0,0)

        self.roll_container = None
        self.rc_dead_time   = TIME_LOAD_BUFFER_LANE

    def set_color(self, col):
        self.color = col

    def set_name(self, name):
        self.name = name

    def set_truck_list(self, truck_list):
        self.truck_list = sorted([truck for truck in truck_list if truck.dock==self.dock], key=lambda t: t.arrival)

    def draw(self, floor_plan):
        if self.truck is None:
            pt1 = floor_plan.pnt_from_coords(self.w1, self.h2-0.15)
            pt2 = floor_plan.pnt_from_coords(self.w2, self.h1)

            floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, self.color, -1)

            pnt  = floor_plan.pnt_from_coords(self.w1 + 0.05 * W_DOCK, 0.8 * self.h1)
            font = cv.FONT_HERSHEY_SIMPLEX
            floor_plan.figure = cv.putText(floor_plan.figure, self.name, pnt, font, 0.4, WHITE)

        else:
            if self.truck.inbound:
                w                 = self.w2
                pt1               = floor_plan.pnt_from_coords(w, 0.9 * self.h1 + 0.1 * self.h2)
                pt2               = floor_plan.pnt_from_coords(w, 0.1 * self.h1 + 0.9 * self.h2)
                floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt1, pt2, (200, 200, 200), 2, tipLength=0.3)
                self.truck.draw(floor_plan, self)
            else:
                w                 = self.w1
                pt1               = floor_plan.pnt_from_coords(w, 0.1 * self.h1 + 0.9 * self.h2)
                pt2               = floor_plan.pnt_from_coords(w, 0.9 * self.h1 + 0.1 * self.h2)
                floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt1, pt2, (200, 200, 200), 2, tipLength=0.3)
                self.truck.draw(floor_plan, self)

        pt1 = floor_plan.pnt_from_coords(0.95*self.w1+0.05*self.w2,-0.05*self.h1+0.95*self.h2)
        pt2 = floor_plan.pnt_from_coords(0.05*self.w1+0.95*self.w2,-0.95*self.h1+0.05*self.h2)
        cv.rectangle(floor_plan.figure, pt1, pt2, WHITE, -1)
        if self.roll_container:
            self.roll_container.draw(floor_plan)

    def get_n_trucks_todo(self, inbound=True):
        todo  = [] if self.truck is None else [self.truck]
        todo += self.truck_list
        return len([t for t in todo if t.inbound==inbound])

    def time_step(self, floor_plan):
        if self.truck:
            self.truck.time_step()

        time_sec = floor_plan.time_sec
        if len(self.truck_list)>0 and time_sec>self.truck_list[0].arrival:
            self.truck = self.truck_list.pop(0)
            self.truck.start_docking()

        if self.roll_container:
            self.rc_dead_time -= TIME_STEP_S

        if self.truck is None:
            return

        self.truck.time_step()

        if self.truck.can_be_undocked():
            self.truck.undock()
            self.truck = None
            return

        if self.truck.inbound and self.roll_container is None and self.truck.can_be_unloaded():
            self.put_roll_container(self.truck.unload_next_roll_container(), inbound=True)

        elif not self.truck.inbound and not self.roll_container is None and self.rc_dead_time<=0.:
            self.truck.load_next_roll_container(self.roll_container)
            self.roll_container = None
            self.rc_dead_time   = TIME_LOAD_BUFFER_LANE

    def roll_container_available(self):
        if not self.truck or not self.truck.inbound: return False
        return not self.roll_container is None and self.rc_dead_time<=0.

    def can_roll_container_be_stored(self):
        if not self.truck: return False
        return self.roll_container is None and self.truck.can_be_loaded()

    def get_roll_container(self):
        roll = self.roll_container
        self.roll_container = None
        self.rc_dead_time   = TIME_LOAD_BUFFER_LANE
        return roll

    def put_roll_container(self, rol, inbound=False):
        self.roll_container   = rol
        self.rc_dead_time     = TIME_LOAD_BUFFER_LANE
        self.roll_container.w = 0.2*self.w1 + 0.8*self.w2 if inbound else 0.8*self.w1 + 0.2*self.w2
        self.roll_container.h = -self.h1/2
