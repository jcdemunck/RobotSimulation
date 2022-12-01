from XdockParams import W_BUFFER_COMP, H_BUFFER_COMP, W_DOCK, W_ROLL_CONTAINER, H_LANE_STORE, \
                        H_FRONT, H_MANEUVER, H_RIGHT, H_LEFT, LOG_DIR


def get_log_filename(base_name):
    return LOG_DIR + ModelParams.hash_code + "_"+base_name + ".log"

def get_model_params():
    text = f"{ModelParams.N_BUFFER_STORE=}\n" \
           + f"{ModelParams.N_BUFFER_ROW=}\n" \
           + f"{ModelParams.N_BUFFER_COL=}\n" \
           + f"{ModelParams.MAX_LANE_STORE=}\n" \
           + f"{ModelParams.N_DOCK=}\n" \
           + f"{ModelParams.N_ROBOT=}\n"
    return text

class ModelParams:
    # Independent parameters:
    N_BUFFER_STORE  =  3
    N_BUFFER_ROW    =  8
    N_BUFFER_COL    =  3
    MAX_LANE_STORE  = 12
    N_DOCK          = 16
    N_ROBOT         = 32

    # Dependent parameters, to be set by self.__update()
    W_BUFFER_STORE  = 0.
    H_BUFFER_STORE  = 0.

    W_DOWN          = 0.
    W_UP            = 0.
    H_FLOOR         = 0.
    W_FLOOR         = 0.

    H_LANE          = 0.

    hash_code       = ""
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

        ModelParams.H_LANE  = H_LANE_STORE * ModelParams.MAX_LANE_STORE
        ModelParams.H_FLOOR = H_FRONT + ModelParams.H_LANE + H_MANEUVER + (H_RIGHT + ModelParams.H_BUFFER_STORE + H_LEFT) * ModelParams.N_BUFFER_STORE
        ModelParams.W_FLOOR = ModelParams.N_DOCK * W_DOCK

        ModelParams.hash_code = str(abs(hash(get_model_params())))

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