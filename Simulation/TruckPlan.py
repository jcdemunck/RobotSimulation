import math
import pandas as pd
import numpy as np
import random
import bisect as bi

from Truck import Truck

from XdockParams import N_DOCK, MAX_TRUCK_LOAD, MAX_DOCK_TIME_LOADING, MAX_DOCK_TIME_UNLOADING, MIN_DOCK_TIME_GAP
from RollContainer import RollContainer
from SimulationConfig import PRIO_LIST, DESTINATIONS, destination_color_dict, get_output_dock, destination_from_dock, prio_from_dock

DIR     = "C:/Users/MunckJande/OneDrive - PostNL/Documenten/Projecten/Robots_at_Xdocks/"
FILE_IN = "Transport_summary.xlsx"


class TruckPlan:
    def __init__(self, simulate, x_dock_name="C_TL"):

        self.truck_list = []
        if simulate: self.__simulate_truck_list()
        else:        self.__trucks_from_file(x_dock_name)

        self.__assign_docks()

        self.start_time = self.truck_list[0].arrival
        self.end_time   = self.truck_list[-1].departure

        self.destinations = sorted(set(truck.destination for truck in self.truck_list if not truck.inbound))
        self.prios        = sorted(set(p for truck in self.truck_list if not truck.inbound for p in truck.prios))

    def get_minimum_stock_function(self, time_delta, dest, prio):
        t_min  = time_delta * int(-1.0+self.start_time/time_delta)
        t_max  = time_delta * int( 1.0+self.end_time  /time_delta)
        n_time = int(0.5+(t_max-t_min)/time_delta)

        cum_in  = np.zeros(n_time, int)
        max_out = np.zeros(n_time, int)
        for truck in self.truck_list:
            t = int( (truck.arrival-t_min)/time_delta)
            if truck.inbound:
                cum_in[t]    += len([r for r in truck.truck_load if r.dest==dest and r.prio==prio])
            elif truck.destination==dest:
                max_out[t] += MAX_TRUCK_LOAD

        for t in range(1,n_time):
            cum_in[t]  += cum_in[t-1]
            max_out[t] += max_out[t-1]

        time_func  = np.zeros(n_time, float)
        stock_func = np.zeros(n_time, int)
        for t in range(n_time):
            stock_func[t] = max(0, cum_in[t]-max_out[t])
            time_func[t]  = t_min + t*time_delta

        return stock_func, time_func

    def __simulate_truck_list(self):
        def create_truck(nrc, destination, t_arrive, high_prio=None):
            if destination is None:  # unloading truck
                dest_list = random.choices(DESTINATIONS, weights=[np.exp(n*1.3) for n in range(len(DESTINATIONS))], k=nrc)
                prio_list = random.choices(PRIO_LIST, weights=[np.exp(n*2) for n in range(len(PRIO_LIST))], k=nrc)
                rc_list   = [RollContainer(0., 0., 3, dest, prio, destination_color_dict[dest]) for (dest, prio) in zip(dest_list, prio_list)]
                t_depart  = t_arrive + MAX_DOCK_TIME_UNLOADING
                return Truck(t_arrive, t_depart, (100, 100, 100), roll_containers=rc_list)

            else:  # loading truck
                prios = [PRIO_LIST[0]] if high_prio else [PRIO_LIST[1]]
                t_depart = t_arrive + MAX_DOCK_TIME_LOADING
                return Truck(t_arrive, t_depart, destination_color_dict[destination], destination=destination, prios=prios)

        random.seed(13)
        self.truck_list = [create_truck(48, None, 0.),
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
                           create_truck(0, "HGL", 2000., high_prio=True)]


    def __trucks_from_file(self, x_dock, max_in=-1, max_out=-1):
        df = pd.read_excel(DIR+FILE_IN)
        df = df[df["x_dock"]==x_dock].sort_values(["time"]).reindex()
        if max_in <=0: max_in  = 1+len(df)
        if max_out<=0: max_out = 1+len(df)

        self.truck_list = []
        n_in            = 0
        n_out           = 0
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
                self.truck_list.append( Truck(t_arrive, t_depart, (100, 100, 100), roll_containers=rc_list, ID=row["trip"]))
                n_in +=1

            elif row.IO=="out_bound" and n_out<=max_out:
                dest     = row["destination"]
                shift    = row["shift"]
                if dest not in DESTINATIONS or shift not in [1,2]: continue
                prios    = [PRIO_LIST[0]] if shift==1 else [PRIO_LIST[1]]
                t_depart = t_arrive + MAX_DOCK_TIME_LOADING
                self.truck_list.append(Truck(t_arrive, t_depart, destination_color_dict[dest], destination=dest, prios=prios, ID=row["trip"]))
                n_out +=1

            if n_in>max_in and n_out>max_out:
                break

    def __assign_docks(self):
        """
            Preliminary dock assignment.
        """
        def find_dock(arrival, departure):
            # find dock with smallest time gap
            gap   = math.inf
            d_min = -1
            for d in range(N_DOCK):
                intervals = [(triple[2].arrival, triple[2].departure) for triple in dock_dict[d]]
                if len(intervals)==0:  return d

                if departure<intervals[0][0]:
                    if gap>intervals[0][0]-departure:
                        gap   = intervals[0][0]-departure
                        d_min = d

                if intervals[-1][1]<arrival:
                    if gap>arrival-intervals[-1][1]:
                        gap   = arrival-intervals[-1][1]
                        d_min = d

                for bb, ee in zip(intervals, intervals[1:]):
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
                if t1.departure<t2.arrival: # No overlap
                    groups.append(group)
                    group   = [t2]
                else:
                    group.append(t2)
                t1 = t2
            groups.append(group)

##            indices = [[trucks.index(t) for t in o] for o in groups]
##            print(destination_from_dock(dock), prio_from_dock(dock), "\t", indices)

            if len(trucks)==len(groups): # all groups are singletons
                return

            # Shift intervals such that none is overlapping with another. Eventually, each group should contain a single element
            for g, group in enumerate(groups):
                if len(group)<=1: continue # OK, no overlap with other intervals

                # t_min, t_max: available time range for each group
                t_min = -math.inf if g==0             else groups[g-1][-1].departure + MIN_DOCK_TIME_GAP
                t_max =  math.inf if g==len(groups)-1 else groups[g+1][ 0].arrival - MIN_DOCK_TIME_GAP

                if t_min==-math.inf and t_max==math.inf:
                    t_req   = sum(t.departure - t.arrival + MIN_DOCK_TIME_GAP for t in group) - MIN_DOCK_TIME_GAP
                    t_begin = (group[0].arrival + group[-1].departure - t_req)/ 2
                    for t in group:
                        t_dock      = t.departure - t.arrival
                        t.arrival   = t_begin
                        t.departure = t_begin + t_dock
                        t_begin     = t.departure + MIN_DOCK_TIME_GAP

                elif t_min==-math.inf:
                    t_end = (t_max + group[-1].departure) / 2
                    for t in reversed(group):
                        t_dock      = t.departure-t.arrival
                        t.departure = t_end
                        t.arrival   = t_end-t_dock
                        t_end       = t.arrival - MIN_DOCK_TIME_GAP

                elif t_max==math.inf:
                    t_begin = (t_min +group[0].arrival) / 2
                    for t in group:
                        t_dock      = t.departure - t.arrival
                        t.arrival   = t_begin
                        t.departure = t_begin + t_dock
                        t_begin     = t.departure + MIN_DOCK_TIME_GAP
                else:
                    t_req   = sum(t.departure - t.arrival + MIN_DOCK_TIME_GAP for t in group) - MIN_DOCK_TIME_GAP
                    t_begin = (t_min + t_max - t_req) / 2
                    for t in group:
                        t_dock      = t.departure - t.arrival
                        t.arrival   = t_begin
                        t.departure = t_begin + t_dock
                        t_begin     = t.departure + MIN_DOCK_TIME_GAP

                # shift previous intervals backwards if enlarged spacing causes overlap
                if g>0 and groups[g-1][-1].departure+MIN_DOCK_TIME_GAP>group[0].arrival:
                    shift = groups[g-1][-1].departure + MIN_DOCK_TIME_GAP - group[0].arrival
                    for gg in range(g):
                        for t in groups[gg]:
                            t.arrival   -= shift
                            t.departure -= shift

                # shift subsequent intervals forward if enlarged spacing causes overlap
                if g<len(groups)-1 and group[-1].departure+MIN_DOCK_TIME_GAP>groups[g + 1][0].arrival:
                    shift = group[-1].departure + MIN_DOCK_TIME_GAP - groups[g + 1][0].arrival
                    for gg in range(g+1, len(groups)):
                        for t in groups[gg]:
                            t.arrival   += shift
                            t.departure += shift

        # First sort all intervals based on arrival times
        self.truck_list.sort(key=lambda t: t.arrival)

        # Plan departing trucks (that need specific dock)
        # Each value of dock_dict contains a list of ordered triples: (arrival, unique integer, truck)
        dock_dict = dict([(d, []) for d in range(N_DOCK)])
        for truck in self.truck_list:
            if truck.inbound: continue

            truck.dock = get_output_dock(truck.destination, truck.prios)
            bi.insort(dock_dict[truck.dock], (truck.arrival, len(dock_dict[truck.dock]), truck))

        # test outbound planning
        for dock in range(N_DOCK):
            trucks = [t for (a,d,t) in dock_dict[dock]]
            if len(trucks)<=0: continue
            print(destination_from_dock(dock), prio_from_dock(dock), "\t", "Range before correction: ",trucks[0].arrival/3600,'-',trucks[-1].departure/3600)
            correct_overlap(trucks)
            print(destination_from_dock(dock), prio_from_dock(dock), "\t", "Range after correction: ",trucks[0].arrival/3600,'-',trucks[-1].departure/3600)

        # put incoming trucks in empty holes
        remove_list = []
        for t,truck in enumerate(self.truck_list):
            if truck.inbound:
                truck.dock = find_dock(truck.arrival, truck.departure)
                if truck.dock<0:
                    print("ERROR. assign_docks(). No dock found for inbound: ", t,"of", len(self.truck_list), "\t",truck)
                    remove_list.append(truck)
                    continue

                bi.insort(dock_dict[truck.dock], (truck.arrival, len(dock_dict[truck.dock]), truck))

        for t in remove_list:
            self.truck_list.remove(t)

        self.test_planning()

    def test_planning(self):
        for dock in range(N_DOCK):
            trucks = [t for t in self.truck_list if t.dock==dock]
            print("dock = ", dock, destination_from_dock(dock), prio_from_dock(dock))

            n_error = 0
            if len(trucks)<=0:
                print("no trucks")
                continue
            t_old = trucks[0]
            if t_old.arrival>=t_old.departure:
                n_error +=1
                print("ERROR. Inconsistent dep/arr", t_old)

            for i,t in enumerate(trucks[1:], start=1):
                if t.arrival>=t.departure:
                    n_error += 1
                    print("ERROR. Inconsistent dep/arr", t)

                if t_old.departure>=t.arrival:
                    n_error += 1
                    print("ERROR. Overlapping dep/arr. gap=", t.arrival-t_old.departure,"i=", i)
                t_old = t
            if n_error==0:
                print("OK")


def main():
    P = TruckPlan(False)
    time_delta = 600.
    _, times = P.get_minimum_stock_function(time_delta, P.destinations[0], P.prios[0])

    df = pd.DataFrame(columns=["destination", "prio"]+list(times/3600))
    for dest in P.destinations:
        for p in P.prios:
            fu, _ = P.get_minimum_stock_function(time_delta, dest, p)
            df.loc[len(df)] = [dest, p] + list(fu)

    df.to_excel(DIR + "Stocks.xlsx")

if __name__=="__main__":
    main()