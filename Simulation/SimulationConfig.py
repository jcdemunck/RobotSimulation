
from XdockParams import N_COMP_Y, N_DOCK
from Position import Position


PRIO_LIST               = ['A','B']
MAX_PRIO                = len(PRIO_LIST)
DESTINATIONS            = ["ALR","AP","KHM","UT"]# + ["LW", "ZL", "HGL", "ELT"]
destination_color_dict  = {"ALR": (0,255,200), "AP": (255,200,0), "KHM":(100,100,255), "UT":(100,0,50),
                           "LW" : (255,0,0),   "ZL": (255,255,0), "HGL":(0,255,200),   "ELT":(0,0,255)}

def set_dock_names_colors(floor_plan):
    for dock in range(N_DOCK):
        dest = destination_from_dock(dock)
        prio = prio_from_dock(dock)

        floor_plan.docks[dock].set_color(destination_color_dict[dest])
        floor_plan.docks[dock].set_name(dest+"-"+prio)

def destination_from_dock(dock):
    return DESTINATIONS[int(dock//MAX_PRIO)]

def prio_from_dock(dock):
    return PRIO_LIST[dock%MAX_PRIO]

def get_output_dock(dest, prio):
    if type(prio)==list:
        prio = prio[0]
    d = DESTINATIONS.index(dest)
    p = PRIO_LIST.index(prio)
    return d*MAX_PRIO+p

def choose_store(floor_plan, roll_container):
    dock = get_output_dock(roll_container.dest, roll_container.prio)
    prio = roll_container.prio
    lane = floor_plan.get_available_output_lane(dock)
    if lane<0:
        buffer_store = floor_plan.buffer_stores[(dock, 0)]
        ##if prio==PRIO_LIST[1]: row = buffer_store.get_first_available_store(int(N_COMP_Y//2), N_COMP_Y)
        ##else:                  row = buffer_store.get_first_available_store(0, int(N_COMP_Y/2))
        row = buffer_store.get_first_available_store(0, N_COMP_Y)
        if row<0:
            print("ERROR: SimulationConfig.choose_store(). buffer overflow. prio = ", prio, "dock = ", dock)
        pos_store = Position(floor_plan, dock, buffer_store=0, row=row, col=0)
        buffer_store.reserve_store(row)
    else:
        pos_store = Position(floor_plan, dock, buffer_lane=lane)
        floor_plan.buffer_lanes[(dock, lane)].reserve_store()

    return pos_store
