import numpy as np

EPS   = 1.e-6
WHITE = (255,255,255)
BLACK = (0,0,0)

H_COMPARTMENT  = 0.9
W_COMPARTMENT  = 0.9
N_COMP_X       = 4 ## 3
N_COMP_Y       = 12
N_BUFFER_STORE = 1 ##2
W_BUFFER_STORE = N_COMP_X*W_COMPARTMENT
H_BUFFER_STORE = N_COMP_Y*H_COMPARTMENT


H_FRONT = 2.0
H_REAR  = 2.0

W_DOWN         = 0.8
W_MID          = 0.##0.5
W_UP           = 0.8
W_DOCK         = W_DOWN + N_BUFFER_STORE*W_BUFFER_STORE + W_MID +W_UP

H_PARK         = 2.0
W_PARK_PLACE   = 1.1
H_RIGHT        = 2.0
H_LEFT         = 2.0
H_MANEUVER     = H_RIGHT + H_PARK + H_LEFT

N_LANE         =  4   # For each dock: two up and two down
W_LANE         =  1.1
H_LANE         = 18.
MAX_LANE_STORE = 24
H_LANE_STORE   = 0.75 # should be H_LANE/MAX_LANE_STORE and this should be larger than H_ROLL_CONTAINER

N_DOCK     =  8

W_FLOOR    = N_DOCK * W_DOCK
H_FLOOR    = H_FRONT + H_LANE + H_MANEUVER + N_COMP_Y * H_COMPARTMENT + H_REAR

H_ROLL_CONTAINER = 0.7
W_ROLL_CONTAINER = 0.7
TIME_ROLL_CONTAINER_LOAD = 2.5 # >= H_ROLL_CONTAINER/BUFFER_LANE_SPEED

W1_ROBOT = 0.3
W2_ROBOT = 0.9
H1_ROBOT = 0.7
H2_ROBOT = 0.4

TIME_STEP_S       =  0.3
ROBOT_SPEED       =  1.0  # (1.2) m/s
ROBOT_LOAD_TIME   = 5. # [s]
ROBOT_UNLOAD_TIME = 5. # [s]
BUFFER_LANE_SPEED =  0.3

MAX_TRUCK_LOAD    = 48
TRUCK_LOAD_TIME   = 10.# [s]
DOCK_TIME         = 500. # [s]

_CO_SCALE = 1000.
def round_coord(co):
    return int(_CO_SCALE*co+0.5)/_CO_SCALE
def round_coords(co):
    return int(_CO_SCALE*co[0]+0.5)/_CO_SCALE, int(_CO_SCALE*co[1]+0.5)/_CO_SCALE

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
