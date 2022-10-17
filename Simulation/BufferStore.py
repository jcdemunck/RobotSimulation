import cv2 as cv
from FloorPlan import round_coords, \
                      N_DOCK, W_DOCK, H_LANE, H_FLOOR, H_FRONT, H_LEFT, W_UP, W_DOWN, H_MANEUVER,\
                      N_BUFFER_STORE, W_BUFFER_STORE,  H_BUFFER_STORE, N_COMP_X, N_COMP_Y, W_COMPARTMENT, H_COMPARTMENT, \
                      BLACK

class BufferStore:
    def __init__(self, dock, buffer):
        if dock<0   or N_DOCK<=dock: return
        if buffer<0 or N_BUFFER_STORE<=buffer: return

        self.dock   = dock
        self.buffer = buffer
        self.h1     = H_FRONT + H_LANE + H_MANEUVER
        self.h2     = self.h1 + H_BUFFER_STORE
        if N_BUFFER_STORE==1:
            self.w1  = dock*W_DOCK + (W_DOWN+W_DOCK-W_UP-W_BUFFER_STORE)/2
            self.w2  = self.w1 + W_BUFFER_STORE

            # surrounding path coords
            self.w1_ext =  dock   *W_DOCK + W_DOWN/2
            self.w2_ext = (dock+1)*W_DOCK - W_UP/2

        else:
            step     = (W_DOCK - W_DOWN-W_UP - W_BUFFER_STORE)/(N_BUFFER_STORE-1)
            self.w1  = W_DOWN + dock*W_DOCK + buffer * step
            self.w2  = self.w1 + W_BUFFER_STORE

            # surrounding path coords
            if buffer==0:                 self.w1_ext = self.w1 - (self.w1-dock*W_DOCK)/2
            else:                         self.w1_ext = self.w1 - (step-W_BUFFER_STORE)/2

            if buffer==N_BUFFER_STORE-1:  self.w2_ext = self.w2 + ((dock+1)*W_DOCK-self.w2)/2
            else:                         self.w2_ext = self.w2 + (step-W_BUFFER_STORE)/2

        self.h1_ext = self.h1 - H_LEFT / 2
        self.h2_ext = self.h2 + (H_FLOOR-self.h2) / 2

        self.w1, self.h1         = round_coords((self.w1, self.h1))
        self.w2, self.h2         = round_coords((self.w2, self.h2))
        self.w1_ext, self.h1_ext = round_coords((self.w1_ext, self.h1_ext))
        self.w2_ext, self.h2_ext = round_coords((self.w2_ext, self.h2_ext))

        self.store = [[] for row in range(N_COMP_Y)]

    def store_roll_container(self, row, rol):
        if row>=N_COMP_Y: return
        if len(self.store[row])>=N_COMP_X: return

        col  = len(self.store[row])
        rol.w, rol.h = self.get_grid_coords(row=row, col=col)
        rol.o        = 2

        self.store[row].append(rol)

    def get_grid_coords(self, row=-1, col=-1, corner=-1):
        if row<0 or col<0:
            if corner==0: return self.w1_ext, self.h1_ext  # lower left
            if corner==1: return self.w2_ext, self.h1_ext  # lower right
            if corner==2: return self.w2_ext, self.h2_ext  # upper right
            if corner==3: return self.w1_ext, self.h2_ext  # upper left

        else:
            w = self.w1 + (col + 0.5) * W_COMPARTMENT
            h = self.h1 + (row + 0.5) * H_COMPARTMENT
            return round_coords((w, h))

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
            for rol in self.store[row]:
                rol.draw(floor_plan)

    def draw_circulation(self, floor_plan):
        # horizontal, to left
        pt1 = floor_plan.pnt_from_coords(self.w1, self.h2_ext)
        pt2 = floor_plan.pnt_from_coords(self.w2, self.h2_ext)
        floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt2, pt1, (0, 200, 0), 3)

        # vertical, downwards
        pt1 = floor_plan.pnt_from_coords(self.w1_ext, self.h1)
        pt2 = floor_plan.pnt_from_coords(self.w1_ext, self.h2)
        floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt2, pt1, (0, 200, 0), 3)

        # vertical, upwards
        if self.buffer==N_BUFFER_STORE-1:
            pt1 = floor_plan.pnt_from_coords(self.w2_ext, self.h1)
            pt2 = floor_plan.pnt_from_coords(self.w2_ext, self.h2)
            floor_plan.figure = cv.arrowedLine(floor_plan.figure, pt1, pt2, (0, 200, 0), 3)
