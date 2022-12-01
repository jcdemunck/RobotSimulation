from ModelParameters import ModelParams as M


PRIO_LIST               = ['A','B']
MAX_PRIO                = len(PRIO_LIST)
DESTINATIONS            = ["ALR","AP","KHM","UT"] + ["LW", "ZL", "HGL", "ELT"]
destination_color_dict  = {"ALR": (0,255,255), "AP": (255,  0,0), "KHM":(100,100,255), "UT":(100,0,50),
                           "LW" : (255,0,150), "ZL": ( 50,255,0), "HGL":(250,255,  0), "ELT":(0,0,255)}

def set_dock_names_colors(floor_plan):
    for dock in range(M.N_DOCK):
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
