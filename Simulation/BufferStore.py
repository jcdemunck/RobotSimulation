import cv2 as cv
from XdockParams import round_coord, round_coords, \
                      N_DOCK, W_DOCK, H_LANE, H_FLOOR, H_FRONT, H_RIGHT, H_LEFT, W_UP, W_DOWN, H_MANEUVER,\
                      N_BUFFER_STORE, W_BUFFER_STORE,  H_BUFFER_STORE, N_COMP_X, N_COMP_Y, W_COMPARTMENT, H_COMPARTMENT, \
                      BLACK

from SimulationConfig import destination_color_dict, PRIO_LIST
from Robot import BSM

class BufferStoreRow:
    def __init__(self, w_dict, h):
        self.n_store_reserved = 0 # number of places reserved for storage
        self.store            = []
        self.w_dict           = w_dict
        self.h                = h

    def get_n_stored(self):
        return len(self.store)

    def reserve_store(self):
        self.n_store_reserved += 1

    def is_store_available(self):
        return self.n_store_reserved<N_COMP_X

    def store_roll_container(self, rol):
        if len(self.store)>=N_COMP_X:
            return

        # update roll container coordinates
        col   = self.get_n_stored()
        rol.w = self.w_dict[col]
        rol.h = self.h
        rol.o = 2

        self.store.append(rol)
        self.store.sort(key=lambda r:r.scheduled)
        for col, rol in enumerate(self.store):
            rol.w = self.w_dict[col]

    def get_n_not_scheduled(self):
        return len([r for r in self.store if not r.scheduled])

    def schedule_roll_container(self):
        for rol in reversed(self.store):
            if rol.scheduled: continue
            rol.scheduled = True
            return

    def pickup_roll_container(self):
        if len(self.store)<=0:
            return

        self.n_store_reserved -= 1
        return self.store.pop(-1)

class BufferStore:
    def __init__(self, dock, buffer):
        if dock<0   or N_DOCK<=dock: return
        if buffer<0 or N_BUFFER_STORE<=buffer: return

        self.dock   = dock
        self.buffer = buffer

        self.w1 = dock * W_DOCK + (W_DOWN + W_DOCK - W_UP - W_BUFFER_STORE) / 2
        self.w2 = self.w1 + W_BUFFER_STORE

        self.h1 = H_FRONT + H_LANE + H_MANEUVER + H_RIGHT + buffer * (H_BUFFER_STORE + H_LEFT + H_RIGHT)
        self.h2 = self.h1 + H_BUFFER_STORE

        # surrounding path coords
        self.w1_ext = dock * W_DOCK + W_DOWN / 2
        self.w2_ext = (dock + 1) * W_DOCK - W_UP / 2

        self.h1_ext = self.h1 - H_RIGHT / 2
        self.h2_ext = self.h2 + H_LEFT  / 2

        self.w1, self.h1         = round_coords((self.w1, self.h1))
        self.w2, self.h2         = round_coords((self.w2, self.h2))
        self.w1_ext, self.h1_ext = round_coords((self.w1_ext, self.h1_ext))
        self.w2_ext, self.h2_ext = round_coords((self.w2_ext, self.h2_ext))

        self.w_dict = dict([(col, round_coord(self.w1 + (col+0.5)*W_COMPARTMENT)) for col in range(N_COMP_X)])
        self.h_dict = dict([(row, round_coord(self.h1 + (row+0.5)*H_COMPARTMENT)) for row in range(N_COMP_Y)])
        self.store  = [BufferStoreRow(self.w_dict, self.h_dict[row]) for row in range(N_COMP_Y)]

    def is_store_unused(self):
        for row_store in self.store:
            if row_store.n_store_reserved>0: return False
        return True

    def schedule_roll_container(self, row):
        self.store[row].schedule_roll_container()

    def get_row_not_scheduled(self):
        for row in range(N_COMP_Y):
            if self.store[row].get_n_not_scheduled()>0:
                return row
        return -1

    def get_first_available_store(self):
        for row in range(N_COMP_Y):
            if self.store[row].is_store_available():
                return row
        return -1

    def reserve_store(self, row):
        self.store[row].reserve_store()

    def store_roll_container(self, row, rol):
        if row>=N_COMP_Y: return
        if self.store[row].get_n_stored()>=N_COMP_X: return

        self.store[row].store_roll_container(rol)

    def pickup_roll_container(self, row):
        if row>=N_COMP_Y: return
        return self.store[row].pickup_roll_container()

    def get_grid_coords(self, row=-1, col=-1, corner=-1):
        if row<0 or col<0:
            if corner==0: return self.w1_ext, self.h1_ext  # lower left
            if corner==1: return self.w2_ext, self.h1_ext  # lower right
            if corner==2: return self.w2_ext, self.h2_ext  # upper right
            if corner==3: return self.w1_ext, self.h2_ext  # upper left

        else:
            return self.w_dict[col], self.h_dict[row]

    def get_store_positions(self):
        return [self.get_grid_coords(row, 0) for row in range(N_COMP_Y)]

    def get_lowest_row_coords(self, left=True):
        if left: return round_coords((self.w1_ext, self.h1 + 0.5 * H_COMPARTMENT))
        else:    return round_coords((self.w2_ext, self.h1 + 0.5 * H_COMPARTMENT))

    def draw(self, floor_plan):
        for i in range(N_COMP_X):
            w1 = self.w1 + i * W_COMPARTMENT
            w2 = self.w1 + (i + 1) * W_COMPARTMENT
            for j in range(N_COMP_Y):
                h1 = self.h1 + (j + 1) * H_COMPARTMENT
                h2 = self.h1 + j * H_COMPARTMENT

                pt1 = floor_plan.pnt_from_coords(w1, h1)
                pt2 = floor_plan.pnt_from_coords(w2, h2)

                floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, BLACK, 1)

        for row in range(N_COMP_Y):
            for rol in self.store[row].store:
                rol.draw(floor_plan)

        dest, prio = BSM.loc_dp_dict[self.dock, self.buffer]
        col = destination_color_dict[dest]
        lt  = 1 if prio==PRIO_LIST[0] else 2
        pt1 = floor_plan.pnt_from_coords(self.w1, self.h1)
        pt2 = floor_plan.pnt_from_coords(self.w2, self.h2)
        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, col, lt)

    def draw_circulation(self, floor_plan):
        # horizontal, to left
        pt1 = floor_plan.pnt_from_coords(self.w1, self.h2_ext)
        pt2 = floor_plan.pnt_from_coords(self.w2, self.h2_ext)
        floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt2, pt1, (0, 200, 0), 3)

        # horizontal, right
        pt1 = floor_plan.pnt_from_coords(self.w2, self.h1_ext)
        pt2 = floor_plan.pnt_from_coords(self.w1, self.h1_ext)
        floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt2, pt1, (0, 200, 0), 3)

        # vertical, downwards
        pt1 = floor_plan.pnt_from_coords(self.w1_ext, self.h1)
        pt2 = floor_plan.pnt_from_coords(self.w1_ext, self.h2)
        floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt2, pt1, (0, 200, 0), 3)

        # vertical, upwards
        pt1 = floor_plan.pnt_from_coords(self.w2_ext, self.h1)
        pt2 = floor_plan.pnt_from_coords(self.w2_ext, self.h2)
        floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt1, pt2, (0, 200, 0), 3)
