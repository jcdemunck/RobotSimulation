import math
import pandas as pd
import numpy as np
import random
import bisect as bi

from Truck import Truck

from XdockParams import N_DOCK, MAX_TRUCK_LOAD, TIME_LOAD_RC_TRUCK, TIME_UNLOAD_RC_TRUCK, TIME_DOCK_INBOUND, TIME_DOCK_OUTBOUND
from RollContainer import RollContainer
from SimulationConfig import PRIO_LIST, DESTINATIONS, destination_color_dict, get_output_dock, destination_from_dock, prio_from_dock

random.seed(13)

MAX_DOCK_TIME_LOADING   = TIME_DOCK_INBOUND + MAX_TRUCK_LOAD * TIME_LOAD_RC_TRUCK + TIME_DOCK_OUTBOUND
MAX_DOCK_TIME_UNLOADING = TIME_DOCK_INBOUND + MAX_TRUCK_LOAD * TIME_UNLOAD_RC_TRUCK + TIME_DOCK_OUTBOUND

def get_truck_list():
    def create_truck(nrc, destination, t_arrive, high_prio=None):
        if destination is None:  # unloading truck
            dest_list = random.choices(DESTINATIONS, weights=[np.exp(n*0.6) for n in range(len(DESTINATIONS))], k=nrc)
            prio_list = random.choices(PRIO_LIST, weights=[np.exp(n) for n in range(len(PRIO_LIST))], k=nrc)
            rc_list   = [RollContainer(0., 0., 3, dest, prio, destination_color_dict[dest]) for (dest, prio) in zip(dest_list, prio_list)]
            t_depart  = t_arrive + MAX_DOCK_TIME_UNLOADING
            return Truck(t_arrive, t_depart, (100, 100, 100), roll_containers=rc_list)

        else:  # loading truck
            prios = [PRIO_LIST[0]] if high_prio else [PRIO_LIST[1]]
            t_depart = t_arrive + MAX_DOCK_TIME_LOADING
            return Truck(t_arrive, t_depart, destination_color_dict[destination], destination=destination, prios=prios)

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
            t_depart = t_arrive + MAX_DOCK_TIME_UNLOADING
            truck_list.append( Truck(t_arrive, t_depart, (100, 100, 100), roll_containers=rc_list, ID=row["trip"]))
            n_in +=1

        elif row.IO=="out_bound" and n_out<=max_out:
            dest     = row["destination"]
            shift    = row["shift"]
            if dest not in DESTINATIONS or shift not in [1,2]: continue
            prios    = [PRIO_LIST[0]] if shift==1 else [PRIO_LIST[1]]
            t_depart = t_arrive + MAX_DOCK_TIME_LOADING
            truck_list.append(Truck(t_arrive, t_depart, destination_color_dict[dest], destination=dest, prios=prios, ID=row["trip"]))
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

    def correct_overlap(trucks):
        # Detect groups of overlapping docking time intervals
        t1      = trucks[0]
        groups  = []
        group   = [t1]
        for t2 in trucks[1:]:
            if t1.departure<=t2.arrival:
                groups.append(group)
                group   = [t2]
            else:
                group.append(t2)
            t1 = t2
        groups.append(group)

###        indices = [[trucks.index(t) for t in o] for o in groups]
###        print(destination_from_dock(dock), prio_from_dock(dock), "\t", indices)
        if len(trucks)==len(groups):
            return

        # Correct time intervals
        for g, group in enumerate(groups):
            if len(group)<=1: continue

            t_min = -math.inf if g==0 else groups[g-1][-1].departure
            t_max =  math.inf if g==len(groups)-1 else groups[g+1][0].arrival

            if t_min==-math.inf and t_max==math.inf:
                t_req   = sum(t.departure - t.arrival for t in group)
                t_begin = (group[0].arrival + group[-1].departure - t_req) / 2
                for t in group:
                    t_dock      = t.departure - t.arrival
                    t.arrival   = t_begin
                    t.departure = t_begin + t_dock
                    t_begin     = t.departure
            elif t_min==-math.inf:
                t_end = (t_max + group[-1].departure) / 2
                for t in reversed(group):
                    t_dock      = t.departure-t.arrival
                    t.departure = t_end
                    t.arrival   = t_end-t_dock
                    t_end       = t.arrival
            elif t_max==math.inf:
                t_begin = (t_min +group[0].arrival) / 2
                for t in group:
                    t_dock      = t.departure - t.arrival
                    t.arrival   = t_begin
                    t.departure = t_begin + t_dock
                    t_begin     = t.departure
            else:
                t_req   = sum(t.departure - t.arrival for t in group)
                t_begin = (t_min + t_max - t_req) / 2
                for t in group:
                    t_dock      = t.departure - t.arrival
                    t.arrival   = t_begin
                    t.departure = t_begin + t_dock
                    t_begin     = t.departure
        # Test
        correct_overlap(trucks)

#    for dest in DESTINATIONS:
#        for prio in PRIO_LIST:
#            tl = sorted([truck for truck in truck_list if truck.destination==dest and prio in truck.prios], key=lambda t: t.arrival)
#            if len(tl)<2: continue
#            print(dest, "\t", prio, "\trange:", tl[0].arrival / 3600, "-",tl[-1].departure / 3600)

    # first plan departing trucks (that need specific dock)
    dock_dict = dict([(d, []) for d in range(N_DOCK)])
    for truck in sorted(truck_list, key=lambda tr: tr.arrival):
        if truck.inbound: continue

        truck.dock = get_output_dock(truck.destination, truck.prios)
        bi.insort(dock_dict[truck.dock], (truck.arrival, len(dock_dict[truck.dock]), truck))

    # test outbound planning
    for dock in range(N_DOCK):
        trucks = [t for (a,d,t) in dock_dict[dock]]
        print(destination_from_dock(dock), prio_from_dock(dock), "\t", "Range before correction: ",trucks[0].arrival/3600,'-',trucks[-1].departure/3600)
        correct_overlap(trucks)
        print(destination_from_dock(dock), prio_from_dock(dock), "\t", "Range after correction: ",trucks[0].arrival/3600,'-',trucks[-1].departure/3600)

    # put incoming trucks in empty holes
    for t,truck in enumerate(truck_list):
        if truck.inbound:
            truck.dock = find_dock(truck.arrival, truck.departure)
            if truck.dock<0:
                print("ERROR. assign_docks(). No dock found for inbound: ", t, "\t",truck)
                print("\toccupation")
                for d in range(N_DOCK):
                    print("\t",dock_dict[d])

            bi.insort(dock_dict[truck.dock], (truck.arrival, len(dock_dict[truck.dock]), truck))


def main():
    truck_list = trucks_from_file()

if __name__=="__main__":
    main()