import cv2 as cv
from FloorPlan import get_orientation, get_distance_city_block, get_path_len, \
                      TIME_STEP_S, W_FLOOR, H_FLOOR, \
                      ROBOT_SPEED, ROBOT_LOAD_TIME, ROBOT_UNLOAD_TIME, W1_ROBOT, W2_ROBOT, H1_ROBOT, H2_ROBOT



class Robot:
    def __init__(self, w, h, orientation):
        self.w   = w
        self.h   = h
        self.o   = orientation
        self.rot = [[1, 0], [0, 1]]
        self.col = (200, 0, 0)
        self.rol = None

        self.__update_rotmat()

        self.sample    = 0
        self.path      = [(self.w, self.h)]
        self.offset    = 0.
        self.segment   = 0

        self.task_list = []

    def set_coords(self, pos):
        self.w, self.h = pos
        if self.rol:
            self.rol.w, self.rol.h = pos
            self.rol.o = self.o

    def set_path(self, path):
        self.task_list = [path]
        self.init_path(path)

    def init_path(self, path):
        if len(path)<=0:
            print("ERROR: Robot.init_path(). Invalid length")
            return
        self.path      = path
        self.set_coords(path[0])

        self.offset    = 0.
        self.segment   = 0
        if len(self.path)>1:
            self.o = get_orientation(self.path[0], self.path[1])

    def time_step(self):
        def get_p_offset(p, s):
            if self.o==0 or self.o==2:    return p[0] + (s if self.o==0 else -s), p[1]
            if self.o==1 or self.o==3:    return p[0], p[1] + (s if self.o==1 else -s)

        self.sample += 1
        if len(self.task_list)<=0: return

        if type(self.task_list[0])==float: # waiting time for (un)loading
            if self.task_list[0]>=0.:
                self.task_list[0] -= TIME_STEP_S
            else:
                self.task_list.pop(0)
                if len(self.task_list)>=0 and type(self.task_list[0])==list:
                    self.init_path(self.task_list[0])
                return

        elif type(self.task_list[0])==list: # run over path
            if self.is_at_end_point():
                self.task_list.pop(0)
                return

            if self.segment>=len(self.path)-1:
                return

            step_todo = TIME_STEP_S*ROBOT_SPEED
            p_begin   = get_p_offset(self.path[self.segment], self.offset)
            for p_end in self.path[self.segment+1:]:
                d      = get_distance_city_block(p_begin, p_end)
                if d>0.:
                    self.o = get_orientation(p_begin, p_end)
                    self.__update_rotmat()

                if step_todo<d: # step before segment end point
                    self.offset    += step_todo
                    self.set_coords(get_p_offset(self.path[self.segment], self.offset))
                    return

                else: # step beyond segment end point
                    self.offset  = 0.
                    step_todo   -= d
                    p_begin      = p_end
                    self.segment+=1

            self.segment = len(self.path)  # use this to test whether robot is at end of path
            self.offset  = 0.
            self.set_coords(self.path[-1])

        else: # pickup or store roll container
            store = self.task_list[0]
            if self.rol is None:
                self.rol = store.pickup_roll_container()
            else:
                store.store_roll_container(self.rol)

            self.task_list.pop(0)
            if len(self.task_list)>=0 and type(self.task_list[0])==list:
                self.init_path(self.task_list[0])

    def is_at_end_point(self):
        return self.segment==len(self.path)### or len(self.path)==1

    def get_time_to_pos(self, floor_plan, pos):
        coords1  = (self.w, self.h)
        coords2  = pos.get_coords()
        path     = floor_plan.grid_graph.get_shortest_path(coords1, coords2, get_distance_city_block)

        return get_path_len(path)/ROBOT_SPEED

    def goto_load_store_park(self, floor_plan, pos_load, pos_unload, pos_park):
        if not self.rol is None:
            print("ERROR: RollContainer.goto_load_store_park(). roll container already loaded")
            return

        coords_self   = (self.w, self.h)
        coords_load   = pos_load.get_coords()
        coords_unload = pos_unload.get_coords()
        coords_park   = pos_park.get_coords()

        self.task_list = [floor_plan.grid_graph.get_shortest_path(coords_self, coords_load, get_distance_city_block),
                          pos_load.get_store_object(), ROBOT_LOAD_TIME,
                          floor_plan.grid_graph.get_shortest_path(coords_load, coords_unload, get_distance_city_block),
                          pos_unload.get_store_object(), ROBOT_UNLOAD_TIME,
                          floor_plan.grid_graph.get_shortest_path(coords_unload, coords_park, get_distance_city_block)]

    def __update_rotmat(self):
        if self.o==0:
            self.rot = [[1, 0], [0, 1]]
        elif self.o==1:
            self.rot = [[0, -1], [1, 0]]
        elif self.o==2:
            self.rot = [[-1, 0], [0, -1]]
        elif self.o==3:
            self.rot = [[0, 1], [-1, 0]]

    def __trans_coords(self, w, h):
        """
        return rotated coordinates of point (w,h), with (self.w, self.h) as centre
        """
        x  = w-self.w
        y  = h-self.h
        xa = self.rot[0][0]*x + self.rot[0][1]*y
        ya = self.rot[1][0]*x + self.rot[1][1]*y
        return xa+self.w, ya+self.h

    def draw(self, floor_plan):
        # Engine
        w1  = self.w - 0.5*(W1_ROBOT+W2_ROBOT)
        w2  = w1 + W1_ROBOT
        h1  = self.h - 0.5*H1_ROBOT
        h2  = h1 + H1_ROBOT
        pt1 = floor_plan.pnt_from_coords(*self.__trans_coords(w1, h1))
        pt2 = floor_plan.pnt_from_coords(*self.__trans_coords(w2, h2))

        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, self.col, -1)

        # carrier platform
        w1  = self.w + 0.5*(W1_ROBOT-W2_ROBOT)
        w2  = w1 + W2_ROBOT
        h1  = self.h - 0.5*H2_ROBOT
        h2  = h1 + H2_ROBOT
        pt1 = floor_plan.pnt_from_coords(*self.__trans_coords(w1, h1))
        pt2 = floor_plan.pnt_from_coords(*self.__trans_coords(w2, h2))

        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, self.col, -1)

        if not self.rol is None:
            self.rol.draw(floor_plan)

    def move(self, step):
        if self.o==0:     self.w += step
        elif self.o==1:   self.h += step
        elif self.o==2:   self.w -= step
        elif self.o==3:   self.h -= step

        self.w = max(0., min(W_FLOOR, self.w))
        self.h = max(0., min(H_FLOOR, self.h))
        if not self.rol is None:
            self.rol.w = self.w
            self.rol.h = self.h

    def rotate(self, left):
        step = 1 if left else -1
        self.o = (self.o + step) % 4
        self.__update_rotmat()

        if not self.rol is None:
            self.rol.rotate(left)

    def load_roll_container(self, rol):
        self.rol   = rol
        self.rol.w = self.w
        self.rol.h = self.h
        self.rol.o = self.o

    def unload_roll_container(self):
        rol        = self.rol
        self.rol   = None
        return rol
