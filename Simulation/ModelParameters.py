from XdockParams import W_BUFFER_COMP, H_BUFFER_COMP, W_DOCK, W_ROLL_CONTAINER, H_ROLL_CONTAINER, H_LANE_STORE, \
                        H_FRONT, H_PARK, H_RIGHT, H_LEFT


def get_log_filename(base_name):
    return ModelParams.DATA_DIR + ModelParams.LOG_SUB_DIR + ModelParams.hash_code + "_"+base_name + ".log"

def get_video_dir():
    return ModelParams.DATA_DIR + ModelParams.VIDEO_SUB_DIR

def get_model_params():
    text = f"{ModelParams.N_BUFFER_STORE=}\n" \
           + f"{ModelParams.N_BUFFER_ROW=}\n" \
           + f"{ModelParams.N_BUFFER_COL=}\n" \
           + f"{ModelParams.MAX_LANE_STORE=}\n" \
           + f"{ModelParams.N_DOCK=}\n" \
           + f"{ModelParams.N_PARK_ROW=}\n" \
           + f"{ModelParams.N_ROBOT=}\n" \
           + f"{ModelParams.ROBOT_SPEED=}\n" \
           + f"{ModelParams.ROBOT_LOAD_TIME}\n" \
           + f"{ModelParams.ROBOT_UNLOAD_TIME}\n" \
           + f"{ModelParams.BUFFER_LANE_SPEED}\n"
    return text

class ModelParams:
    # Independent parameters:
    N_BUFFER_STORE  =  3
    N_BUFFER_ROW    =  8
    N_BUFFER_COL    =  3
    MAX_LANE_STORE  = 12
    N_DOCK          = 16
    N_ROBOT         = 32
    N_PARK_ROW      =  2

    ROBOT_SPEED       = 1.0  # (1.2) m/s
    ROBOT_LOAD_TIME   = 5.0
    ROBOT_UNLOAD_TIME = 5.0
    ROBOT_LOGGING     = True

    BUFFER_LANE_SPEED = 0.3

    # Input file/dir
    DATA_DIR        = "C:/Users/MunckJande/OneDrive - PostNL/Documenten/Projecten/Robots_at_Xdocks/"
    LOG_SUB_DIR     = "Logging/"
    VIDEO_SUB_DIR   = "Video/"
    DATA_FILE       = "Transport_summary.xlsx"

    # Dependent parameters, to be set by self.__update()
    W_BUFFER_STORE  = 0.
    H_BUFFER_STORE  = 0.

    W_DOWN          = 0.
    W_UP            = 0.
    H_FLOOR         = 0.
    W_FLOOR         = 0.

    H_LANE          = 0.
    H_MANEUVER      = 0.

    hash_code       = ""

    TIME_LOAD_BUFFER_LANE = 0.

    def __init__(self):
        self.__update()

    def __update(self):
        ModelParams.W_BUFFER_STORE = ModelParams.N_BUFFER_COL * W_BUFFER_COMP
        ModelParams.H_BUFFER_STORE = ModelParams.N_BUFFER_ROW * H_BUFFER_COMP

        ModelParams.W_DOWN = (W_DOCK - ModelParams.W_BUFFER_STORE) / 2
        ModelParams.W_UP   = ModelParams.W_DOWN

        if ModelParams.W_DOWN<W_ROLL_CONTAINER:
            print("ERROR (Fatal). Inconsistent mode parameters: W_DOWN = ", ModelParams.W_DOWN)
            exit(1)

        ModelParams.H_MANEUVER = H_RIGHT + H_PARK*ModelParams.N_PARK_ROW + H_LEFT
        ModelParams.H_LANE     = H_LANE_STORE * ModelParams.MAX_LANE_STORE
        ModelParams.H_FLOOR    = H_FRONT + ModelParams.H_LANE + ModelParams.H_MANEUVER + (H_RIGHT + ModelParams.H_BUFFER_STORE + H_LEFT) * ModelParams.N_BUFFER_STORE
        ModelParams.W_FLOOR    = ModelParams.N_DOCK * W_DOCK

        ModelParams.TIME_LOAD_BUFFER_LANE = H_ROLL_CONTAINER / ModelParams.BUFFER_LANE_SPEED

        ModelParams.hash_code  = str(abs(hash(get_model_params())))

    def set_n_buffer_store(self, n_store):
        ModelParams.N_BUFFER_STORE = n_store
        self.__update()

    def set_n_buffer_row_col(self, n_row, n_col):
        ModelParams.N_BUFFER_ROW = n_row
        ModelParams.N_BUFFER_COL = n_col
        self.__update()

    def set_max_buffer_lane_store(self, max_lane_store):
        ModelParams.MAX_LANE_STORE = max_lane_store
        self.__update()

    def set_n_dock(self, n_dock):
        ModelParams.N_DOCK = n_dock
        self.__update()

    def set_n_robot(self, n_robot):
        ModelParams.N_ROBOT = n_robot
        self.__update()

    def set_n_park_row(self, n_park_row):
        ModelParams.N_PARK_ROW = n_park_row
        self.__update()

    def set_robot_logging(self, set_log):
        ModelParams.ROBOT_LOGGING = set_log

    def set_data_dir(self, data_dir):
        ModelParams.DATA_DIR = data_dir

    def set_data_input_file(self, file):
        file = file.replace('\\','/')
        ModelParams.DATA_FILE = file.split('/')[-1]

    def set_logging_sub_dir(self, sub_dir):
        sub_dir = sub_dir.replace('\\','/')
        ModelParams.LOG_SUB_DIR = sub_dir.split('/')[-1] + '/'

    def set_video_sub_dir(self, sub_dir):
        sub_dir = sub_dir.replace('\\', '/')
        ModelParams.VIDEO_SUB_DIR = sub_dir.split('/')[-1] + '/'

    def set_robot_speed(self, speed):
        ModelParams.ROBOT_SPEED = speed

    def set_robot_load_unload_time(self, t_load, t_unload):
        ModelParams.ROBOT_LOAD_TIME   = t_load
        ModelParams.ROBOT_UNLOAD_TIME = t_unload

    def set_buffer_lane_speed(self, speed):
        ModelParams.BUFFER_LANE_SPEED = speed
        self.__update()