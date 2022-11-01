import random
import pandas as pd

from XdockParams import N_DOCK, DOCK_TIME
from Position import Position
from Truck import Truck
from RollContainer import RollContainer


MAX_PRIO                = 2
PRIO_LIST               = ['A','B']
DESTINATIONS            = ["ALR","AP","KHM","UT"]
destination_color_dict  = {"ALR": (0,255,200), "AP": (255,200,0), "KHM":(100,100,255), "UT":(100,0,50)}

random.seed(13)
def get_truck_list():
    def create_truck(nrc, destination, t_arrive, high_prio=None):
        if destination is None:  # unloading truck
            rc_list = []
            for t in range(nrc):
                dest = DESTINATIONS[random.randint(0, len(DESTINATIONS) - 1)]
                prio = random.choice(PRIO_LIST)
                rc_list.append(RollContainer(0., 0., 3, dest, prio, destination_color_dict[dest]))

            return Truck(t_arrive, t_arrive + DOCK_TIME, (100, 100, 100), roll_containers=rc_list)

        else:  # loading truck
            prios = [PRIO_LIST[0]] if high_prio else [PRIO_LIST[1]]
            return Truck(t_arrive, t_arrive + DOCK_TIME, destination_color_dict[destination], destination=destination, prios=prios)

    return sorted([create_truck(25, None, 5.),
                   create_truck(30, None, 80.),
                   create_truck(20, None, 150.),
                   create_truck(0, "ALR", 420., high_prio=True),
                   create_truck(0, "AP", 360., high_prio=True)], key=lambda tr: tr.arrival)

def trucks_from_file():
    DIR   = "C:/Users/MunckJande/OneDrive - PostNL/Documenten/Projecten/Robots_at_Xdocks/"
    FILE  = "Transport_summary.xlsx"
    XDOCK = "C_TL"
    df = pd.read_excel(DIR+FILE)
    df = df[df["x_dock"]==XDOCK].sort_values(["time"]).reindex()

    truck_list = []
    n_in       = 0
    n_out      = 0
    time_0     = 0
    for i, row in df.iterrows():
        if i==0:
            time_0 = row["time"]

        t_arrive   = (row["time"]-time_0)/3600.
        if row.IO=="in_bound" and n_in<=5:
            rc_list_s  = row["load"].split(';')
            rc_list    = []
            for rol_s in rc_list_s:
                li   = rol_s.split(',')
                dest = li[0][2:-1]
                if dest not in DESTINATIONS: continue
                prio = int(li[1][:-1])
                rc_list.append(RollContainer(0.,0.,0, dest, prio, destination_color_dict[dest]))
            truck_list.append( Truck(t_arrive, t_arrive + DOCK_TIME, (100, 100, 100), roll_containers=random.shuffle(rc_list)))
            n_in +=1
        elif row.IO=="out_bound" and n_out<=5:
            dest  = row["destination"]
            shift = row["shift"]
            if dest not in DESTINATIONS or shift not in [1,2]: continue
            prios = [PRIO_LIST[0]] if shift==1 else [PRIO_LIST[1]]
            truck_list.append(Truck(t_arrive, t_arrive + DOCK_TIME, destination_color_dict[dest], destination=dest, prios=prios))
            n_out +=1

        if n_in>5 or n_out>5:
            break

    return truck_list


def assign_docks(truck_list):
    """
        Preliminary dock assignment.
    """

    def find_dock(arrival, departure):
        for d in range(N_DOCK):
            times = dock_dict[d]
            if len(times)==0 or departure<times[0][0] or times[-1][1]<arrival:
                return d
            for bb, ee in zip(times, times[1:]):
                if bb[1]<arrival and departure<ee[0]:
                    return d
        return -1

    # first plan departing trucks (that need specific dock)
    dock_dict = dict([(d, []) for d in range(N_DOCK)])
    for truck in sorted(truck_list, key=lambda t: t.arrival):
        if truck.destination is None: continue

        truck.dock = DESTINATIONS.index(truck.destination)
        dock_dict[truck.dock].append((truck.arrival, truck.departure))

    # put incoming trucks in empty holes
    for truck in truck_list:
        if truck.destination is None:
            truck.dock = find_dock(truck.arrival, truck.departure)
            dock_dict[truck.dock] = sorted(dock_dict[truck.dock] + [(truck.arrival, truck.departure)],
                                           key=lambda t: t[0])

def choose_store(floor_plan, roll_container):
    prio = roll_container.prio
    dest = roll_container.dest
    dock = DESTINATIONS.index(dest)
    lane = floor_plan.get_available_output_lane(dock)
    if prio==PRIO_LIST[1] or lane<0:
        buffer_store = floor_plan.buffer_stores[(dock, 0)]
        if prio==PRIO_LIST[1]: row = buffer_store.get_first_available_store(5,9)
        else:                  row = buffer_store.get_first_available_store(0,5)
        pos_store = Position(floor_plan, dock, buffer_store=0, row=row, col=0)
        buffer_store.reserve_store(row)
    else:
        pos_store = Position(floor_plan, dock, buffer_lane=lane)
        floor_plan.buffer_lanes[(dock, lane)].reserve_store()

    return pos_store
