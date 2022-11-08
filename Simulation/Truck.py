import cv2 as cv
import math
import pandas as pd
import numpy as np
import random


from XdockParams import MAX_TRUCK_LOAD, N_DOCK, DOCK_TIME
from RollContainer import RollContainer
from SimulationConfig import PRIO_LIST, DESTINATIONS, destination_color_dict, get_output_dock

random.seed(13)

def get_truck_list():
    def create_truck(nrc, destination, t_arrive, high_prio=None):
        if destination is None:  # unloading truck
            rc_list = []
            for t in range(nrc):
                dest = random.choice(DESTINATIONS)
                prio = random.choice(PRIO_LIST)
                rc_list.append(RollContainer(0., 0., 3, dest, prio, destination_color_dict[dest]))

            return Truck(t_arrive, t_arrive + DOCK_TIME, (100, 100, 100), roll_containers=rc_list)

        else:  # loading truck
            prios = [PRIO_LIST[0]] if high_prio else [PRIO_LIST[1]]
            return Truck(t_arrive, t_arrive + DOCK_TIME, destination_color_dict[destination], destination=destination, prios=prios)

    truck_list = sorted([create_truck(48, None, 0.),
                         create_truck(48, None, 80.),
                         create_truck(48, None, 60.),
                         create_truck(48, None, 150.),
                         create_truck(48, None, 150.),
                         create_truck(48, None, 510.),
                         create_truck(48, None, 150.),
                         create_truck(48, None, 510.),
                         create_truck(48, None, 600.),
                         create_truck(48, None, 600.),
                         create_truck(0, "ALR", 700., high_prio=True),
                         create_truck(0, "ALR", 300., high_prio=False),
                         create_truck(0, "ALR",1210., high_prio=True),
                         create_truck(0, "ALR",810., high_prio=False),
                         create_truck(0, "AP", 2000., high_prio=True)], key=lambda tr: tr.arrival)
    assign_docks(truck_list)
    return truck_list

def trucks_from_file(max_in=-1, max_out=-1):
    DIR   = "C:/Users/MunckJande/OneDrive - PostNL/Documenten/Projecten/Robots_at_Xdocks/"
    FILE  = "Transport_summary.xlsx"
    XDOCK = "C_TL"
    df = pd.read_excel(DIR+FILE)
    df = df[df["x_dock"]==XDOCK].sort_values(["time"]).reindex()
    if max_in <=0: max_in  = 1+len(df)
    if max_out<=0: max_out = 1+len(df)

    truck_list = []
    n_in       = 0
    n_out      = 0
    for i, row in df.iterrows():
        t_arrive   = row["time"]*3600.
        if row.IO=="in_bound" and n_in<=max_in:
            rc_list_s  = row["load"].split(';')
            rc_list    = []
            for rol_s in rc_list_s:
                li    = rol_s.split(',')
                dest  = li[0][2:-1]
                shift = int(li[1][:-1])
                if dest not in DESTINATIONS or shift not in [1,2]: continue
                prio = PRIO_LIST[0] if shift==1 else PRIO_LIST[1]
                rc_list.append(RollContainer(0.,0.,0, dest, prio, destination_color_dict[dest]))

            random.shuffle(rc_list)
            truck_list.append( Truck(t_arrive, t_arrive + DOCK_TIME, (100, 100, 100), roll_containers=rc_list))
            n_in +=1

        elif row.IO=="out_bound" and n_out<=max_out:
            dest  = row["destination"]
            shift = row["shift"]
            if dest not in DESTINATIONS or shift not in [1,2]: continue
            prios = [PRIO_LIST[0]] if shift==1 else [PRIO_LIST[1]]
            truck_list.append(Truck(t_arrive, t_arrive + DOCK_TIME, destination_color_dict[dest], destination=dest, prios=prios))
            n_out +=1

        if n_in>max_in and n_out>max_out:
            break

    truck_list.sort(key=lambda t: t.arrival)
    assign_docks(truck_list)
    return truck_list


def assign_docks(truck_list):
    """
        Preliminary dock assignment.
    """
    def find_dock(arrival, departure):
        # find dock with smallest time gap
        gap   = math.inf
        d_min = -1
        for d in range(N_DOCK):
            times = dock_dict[d]
            if len(times)==0:  return d

            if departure<times[0][0]:
                if gap>times[0][0]-departure:
                    gap   = times[0][0]-departure
                    d_min = d
            if times[-1][1]<arrival:
                if gap>arrival-times[-1][1]:
                    gap   = arrival-times[-1][1]
                    d_min = d

            for bb, ee in zip(times, times[1:]):
                if bb[1]<arrival and departure<ee[0]:
                    if gap>arrival-bb[1]:
                        gap   = arrival-bb[1]
                        d_min = d
                    if gap>ee[0]-departure:
                        gap   = ee[0]-departure
                        d_min = d

        return d_min

    # first plan departing trucks (that need specific dock)
    dock_dict = dict([(d, []) for d in range(N_DOCK)])
    for truck in sorted(truck_list, key=lambda t: t.arrival):
        if truck.destination is None: continue

        truck.dock = get_output_dock(truck.destination, truck.prios)
        dock_dict[truck.dock].append((truck.arrival, truck.departure))

    # put incoming trucks in empty holes
    for t,truck in enumerate(truck_list):
        if truck.destination is None:
            truck.dock = find_dock(truck.arrival, truck.departure)
            if truck.dock<0:
                print("ERROR. assign_docks(). No dock found for: ", t, truck)
                print("\toccupation")
                for d in range(N_DOCK):
                    print("\t",dock_dict[d])

            dock_dict[truck.dock] = sorted(dock_dict[truck.dock] + [(truck.arrival, truck.departure)], key=lambda t: t[0])

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