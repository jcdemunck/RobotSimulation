import cv2 as cv
from FloorPlan import get_orientation, get_distance_city_block, TIME_STEP_S, W_FLOOR, H_FLOOR, \
                      ROBOT_SPEED, W1_ROBOT, W2_ROBOT, H1_ROBOT, H2_ROBOT

class Robot:
    def __init__(self, w, h, orientation, color):

        self.w   = w
        self.h   = h
        self.o   = orientation
        self.rot = [[1, 0], [0, 1]]
        self.col = color
        self.rol = None

        self.__update_rotmat()

        self.sample    = 0
        self.path      = [(self.w, self.h)]
        self.offset    = 0.
        self.segment   = 0

    def set_coords(self, pos):
        self.w, self.h = pos
        if self.rol:
            self.rol.w, self.rol.h = pos
            self.rol.o = self.o

    def set_path(self, path):
        self.path      = path
        self.set_coords(path[0])

        self.offset    = 0.
        self.segment   = 0
        if len(self.path)>1:
            self.o = get_orientation(self.path[0], self.path[1])

    def step(self):
        self.sample += 1
        if self.segment>=len(self.path)-1:
            return

        def get_p_offset(p, s):
            if self.o==0 or self.o==2:    return p[0] + (s if self.o==0 else -s), p[1]
            if self.o==1 or self.o==3:    return p[0], p[1] + (s if self.o==1 else -s)

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

        self.segment   = len(self.path)  # use this to test whether robot is at end of path
        self.offset    = 0.
        self.set_coords(self.path[-1])

    def is_at_end_point(self):
        return self.segment == len(self.path)

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
