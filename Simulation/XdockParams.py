import numpy as np

H_ROLL_CONTAINER = 0.7
W_ROLL_CONTAINER = 0.7

# Robot dimensions
W1_ROBOT = 0.3
W2_ROBOT = 0.9
H1_ROBOT = 0.7
H2_ROBOT = 0.4

# Buffer store dimensions
H_BUFFER_COMP  = 0.9
W_BUFFER_COMP  = 0.9

# Buffer lane
N_LANE         =  4   # For each dock: two up and two down
W_LANE         =  1.1
H_LANE_STORE   = 1.05*H_ROLL_CONTAINER # Space occupied by one roll container on the buffer lane

# Dock dimensions
W_DOCK         =  5.  # width [m]
H_DOCK_LEGENDS = -2.2 # Used for drawing dock/truck info on floor plan

# Parking dimensions
H_PARK         = 1.5
W_PARK_PLACE   = 1.1

# Floor dimensions
H_RIGHT    = 1.0
H_LEFT     = 1.0
H_FRONT    = 2.0

TIME_STEP_S       =  0.25

MAX_TRUCK_LOAD           = 48

# Timings
TIME_LOAD_RC_TRUCK       = 19   # time [s], per roll container to load a roll container onto a truck
TIME_UNLOAD_RC_TRUCK     = 15   # time [s], per roll container to unload a roll container from a truck
TIME_DOCK_INBOUND        = 120  # time [s] the driver needs to put his truck at the dock (before loading/unloading)
TIME_DOCK_OUTBOUND       =  60  # time [s] the driver needs drive away from the dock (after loading/unloading)
TIME_DOCK_EXTRA          =  60  # extra time [s] the driver has for (un)loading to wait before the buffer lane is clear/filled
MIN_DOCK_TIME_GAP        =  30  # minimum time interval [s] between two trucks at a dock

MAX_DOCK_TIME_LOADING    = TIME_DOCK_INBOUND + MAX_TRUCK_LOAD * TIME_LOAD_RC_TRUCK   + TIME_DOCK_EXTRA + TIME_DOCK_OUTBOUND
MAX_DOCK_TIME_UNLOADING  = TIME_DOCK_INBOUND + MAX_TRUCK_LOAD * TIME_UNLOAD_RC_TRUCK + TIME_DOCK_EXTRA + TIME_DOCK_OUTBOUND

# Logging
LOG_INTERVAL_ROBOT       = 60.

# General
EPS   = 1.e-6
WHITE = (255,255,255)
BLACK = (0,0,0)


_COORD_SCALE = 1000.
def round_coord(co):
    return int(_COORD_SCALE * co + 0.5) / _COORD_SCALE
def round_coords(co):
    return round_coord(co[0]), round_coord(co[1])

def get_distance(p1, p2):
    return np.sqrt( (p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) )

def get_distance_city_block(p1, p2):
    return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])

def get_path_len(path):
    p_len = 0.
    if len(path)>1:
        p_old = path[0]
        for p in path:
            p_len += get_distance_city_block(p, p_old)
            p_old  = p

    return p_len

def get_orientation(fr, to):
    if abs(fr[1]-to[1])<EPS:  # segment horizontal
        return 0 if to[0]>fr[0] else 2
    else:
        return 1 if to[1]>fr[1] else 3
