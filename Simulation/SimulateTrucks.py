import math
import pandas as pd
import numpy as np
import random
import heapq as hq

from Truck import Truck

from XdockParams import N_DOCK, MAX_TRUCK_LOAD, TIME_LOAD_RC_TRUCK, TIME_UNLOAD_RC_TRUCK, TIME_EXTRA_TRUCK_DOCKING
from RollContainer import RollContainer
from SimulationConfig import PRIO_LIST, DESTINATIONS, destination_color_dict, get_output_dock, destination_from_dock, prio_from_dock

random.seed(13)

def get_truck_list():
    def create_truck(nrc, destination, t_arrive, high_prio=None):
        if destination is None:  # unloading truck
            dest_list = random.choices(DESTINATIONS, weights=[np.exp(n*0.6) for n in range(len(DESTINATIONS))], k=nrc)
            prio_list = random.choices(PRIO_LIST, weights=[np.exp(n) for n in range(len(PRIO_LIST))], k=nrc)
            rc_list   = [RollContainer(0., 0., 3, dest, prio, destination_color_dict[dest]) for (dest, prio) in zip(dest_list, prio_list)]
            return Truck(t_arrive, (100, 100, 100), roll_containers=rc_list)

        else:  # loading truck
            prios = [PRIO_LIST[0]] if high_prio else [PRIO_LIST[1]]
            return Truck(t_arrive, destination_color_dict[destination], destination=destination, prios=prios)

    truck_list = sorted([create_truck(48, None, 0.),
                         create_truck(48, None, 0.),
                         create_truck(48, None, 60.),
                         create_truck(48, None, 60.),
                         create_truck(48, None, 60.),
                         create_truck(48, None, 700.),
                         create_truck(48, None, 150.),
                         create_truck(48, None, 150.),
                         create_truck(48, None, 150.),
                         create_truck(48, None, 150.),
                         create_truck(48, None, 510.),
                         create_truck(48, None, 510.),
                         create_truck(48, None, 510.),
                         create_truck(48, None, 600.),
                         create_truck(48, None, 600.),
                         create_truck(0, "ELT",1500., high_prio=False),
                         create_truck(0, "ELT",3700., high_prio=False),
                         create_truck(0, "ELT",2110., high_prio=True),
                         create_truck(0, "ELT",2600., high_prio=False),
                         create_truck(0, "HGL", 2000., high_prio=True)], key=lambda tr: tr.arrival)
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
            truck_list.append( Truck(t_arrive, (100, 100, 100), roll_containers=rc_list))
            n_in +=1

        elif row.IO=="out_bound" and n_out<=max_out:
            dest  = row["destination"]
            shift = row["shift"]
            if dest not in DESTINATIONS or shift not in [1,2]: continue
            prios = [PRIO_LIST[0]] if shift==1 else [PRIO_LIST[1]]
            truck_list.append(Truck(t_arrive, destination_color_dict[dest], destination=dest, prios=prios))
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
        if truck.inbound: continue

        truck.dock = get_output_dock(truck.destination, truck.prios)
        departure  = truck.arrival + MAX_TRUCK_LOAD*TIME_LOAD_RC_TRUCK + TIME_EXTRA_TRUCK_DOCKING
        hq.heappush(dock_dict[truck.dock], (truck.arrival, departure))

    # test outbound planning
    for dock in range(N_DOCK):
        for (a1, d1), (a2, d2) in zip(dock_dict[dock][0:-1], dock_dict[dock][1:]):
            if d1>a2:
                print("ERROR. assign_docks(). Overlapping outbound trucks. ", destination_from_dock(dock), prio_from_dock(dock))
                print("       (a1, d1) = ", a1,d1, "(a2, d2) = ", a2, d2)


    # put incoming trucks in empty holes
    for t,truck in enumerate(truck_list):
        if truck.inbound:
            departure  = truck.arrival + MAX_TRUCK_LOAD*TIME_UNLOAD_RC_TRUCK + TIME_EXTRA_TRUCK_DOCKING
            truck.dock = find_dock(truck.arrival, departure)
            if truck.dock<0:
                print("ERROR. assign_docks(). No dock found for inbound: ", t, "\t",truck)
                print("\toccupation")
                for d in range(N_DOCK):
                    print("\t",dock_dict[d])

            hq.heappush(dock_dict[truck.dock], (truck.arrival, departure))

