import cv2 as cv

from Position import Position

from FloorPlan import get_orientation, get_distance_city_block, get_path_len, \
                      TIME_STEP_S, W_FLOOR, H_FLOOR, \
                      ROBOT_SPEED, ROBOT_LOAD_TIME, ROBOT_UNLOAD_TIME, W1_ROBOT, W2_ROBOT, H1_ROBOT, H2_ROBOT, \
                      DESTINATIONS

def print_tasks(task_list):
    for t,task in enumerate(task_list):
        print(t, task.task_type)

class RobotTask:
    def __init__(self, wait=-1., floor_plan=None, goto_pos=None, load_store=None, unload_store=None):
        self.task_type = "no_task"
        self.finished  = True
        self.path      = []
        self.goto      = (0.,0.)

        if not floor_plan is None:
            if not goto_pos is None:
                self.goto    = goto_pos.get_coords()
                self.segment = 0
                self.offset  = 0.
                if goto_pos.pos_type=="buffer_lane":
                    self.task_type = "goto_buffer_lane"
                elif goto_pos.pos_type=="buffer_store":
                    self.task_type = "goto_buffer_store"
                elif goto_pos.pos_type=="parking":
                    self.task_type = "goto_parking"
                else:
                    self.task_type = "goto"
                self.finished = False

        elif not load_store is None:
            self.wait      = wait
            self.store     = load_store
            self.task_type = "pickup"
            self.finished  = False

        elif not unload_store is None:
            self.wait      = wait
            self.store     = unload_store
            self.task_type = "unload"
            self.finished  = False

        elif wait>=0.:
            self.wait      = wait
            self.task_type = "wait"
            self.finished  = self.wait<=0.

    def set_start_point(self, floor_plan, coords_from):
        if len(self.path)<=0:
            return
        self.path = floor_plan.grid_graph.get_shortest_path(coords_from, self.path[-1])

    def time_step(self, robot, floor_plan):
        if self.task_type[0:4]=="goto":
            if len(self.path)==0:
                self.path = floor_plan.grid_graph.get_shortest_path((robot.w,robot.h), self.goto)
            elif self.segment>=len(self.path)-1:
                self.path     = []
                self.finished = True
                return

            step_todo = TIME_STEP_S * ROBOT_SPEED
            p_begin   = robot.get_coords_offset(self.path[self.segment], self.offset)
            for p_end in self.path[self.segment + 1:]:
                d = get_distance_city_block(p_begin, p_end)
                if d>0.:
                    robot.o = get_orientation(p_begin, p_end)
                    robot._update_rot_mat()

                if step_todo<d:  # step before segment end point
                    self.offset += step_todo
                    robot.set_coords(robot.get_coords_offset(self.path[self.segment], self.offset))
                    return

                else:  # step beyond segment end point
                    self.offset   = 0.
                    step_todo    -= d
                    p_begin       = p_end
                    self.segment += 1

            self.segment = len(self.path)  # use this to test whether robot is at end of path
            self.offset  = 0.
            robot.set_coords(self.path[-1])
            self.finished = True

        elif self.task_type=="pickup":
            if self.wait<=0:
                robot.load_roll_container(self.store.pickup_roll_container())
                self.finished = True
            else:
                self.wait -= TIME_STEP_S

        elif self.task_type=="unload":
            if self.wait<=0:
                self.store.store_roll_container(robot.unload_roll_container())
                self.finished = True
            else:
                self.wait -= TIME_STEP_S

        elif self.task_type=="wait":
            self.wait -= TIME_STEP_S
            if self.wait<=0.:
                self.finished = True

    def get_time_to_finish(self):
        if self.finished:
            return 0.
        if self.task_type[0:4]=="goto":
            return W_FLOOR/(2*ROBOT_SPEED)
        if self.task_type=="wait":
            return self.wait
        return TIME_STEP_S

class Robot:
    def __init__(self, w, h, parking_pos):
        self.w           = w
        self.h           = h
        self.o           = 3
        self.rot         = [[1, 0], [0, 1]]
        self.col         = (200, 0, 0)
        self.rol         = None
        self.default_pos = parking_pos

        self._update_rot_mat()

        self.task_list = []

    def get_default_pos(self):
        return self.default_pos

    def set_coords(self, pos):
        self.w, self.h = pos
        if self.rol:
            self.rol.w, self.rol.h = pos
            self.rol.o = self.o

    def get_coords_offset(self, coords, s):
        if self.o==0 or self.o==2:    return coords[0] + (s if self.o==0 else -s), coords[1]
        if self.o==1 or self.o==3:    return coords[0], coords[1] + (s if self.o==1 else -s)

    def insert_process_pickup(self, floor_plan):
        shift = self.rol.shift
        dest  = self.rol.dest
        if shift>5:
            pos_store = Position(floor_plan, DESTINATIONS.index(dest), buffer_store=0, row=shift - 5, col=0)
        else:
            pos_store = Position(floor_plan, DESTINATIONS.index(dest), buffer_lane=3)

        task_list = [RobotTask(floor_plan=floor_plan, goto_pos=pos_store),
                     RobotTask(wait=ROBOT_UNLOAD_TIME, unload_store=pos_store.get_store_object(floor_plan))]

        # insert new tasks
        self.task_list = task_list+self.task_list

        # append goto parking
        for ta in reversed(self.task_list):
            if len(ta.path)>0:
                if ta.task_type!="goto_parking":
                    self.task_list.append(RobotTask(floor_plan=floor_plan, goto_pos=self.get_default_pos()))
                break

    def time_step(self, floor_plan):
        if len(self.task_list)<=0: return

        task = self.task_list[0]
        while task.finished:
            self.task_list.pop(0)
            if len(self.task_list)<=0: return
            task = self.task_list[0]

        task.time_step(self, floor_plan)
        if task.finished:
            self.task_list.pop(0)
            if task.task_type=="pickup":
                self.insert_process_pickup(floor_plan)

    def get_time_to_pos(self, floor_plan, pos):
        coords1  = (self.w, self.h)
        coords2  = pos.get_coords()
        path     = floor_plan.grid_graph.get_shortest_path(coords1, coords2, get_distance_city_block)

        return get_path_len(path)/ROBOT_SPEED

    def get_task_list_length(self):
        return len(self.task_list)

    def get_time_to_finish(self):
        time_to_finish = 0.
        for task in self.task_list:
            time_to_finish += task.get_time_to_finish()
        return time_to_finish

    def get_time_to_start_parking(self):
        time_to_finish = 0.
        for task in self.task_list:
            if task.task_type=="goto_parking":
                break
            time_to_finish += task.get_time_to_finish()
        return time_to_finish

    def wait_goto_pickup(self, floor_plan, wait, pos_pickup):
        if not self.rol is None:
            print("ERROR: RollContainer.wait_goto_pickup(). roll container already loaded")
            return

        self.task_list = [RobotTask(wait=wait),
                          RobotTask(floor_plan=floor_plan, goto_pos=pos_pickup),
                          RobotTask(wait=ROBOT_LOAD_TIME, load_store=pos_pickup.get_store_object(floor_plan)),
                          RobotTask(floor_plan=floor_plan, goto_pos=self.default_pos)]

    def append_goto_pickup(self, floor_plan, pos_pickup):
        task_list   = []
        for task in self.task_list:
            if task.task_type=="goto_parking" or task.task_type=="no_task":
                break
            task_list.append(task)

        self.task_list = task_list +  \
                        [RobotTask(floor_plan=floor_plan, goto_pos=pos_pickup),
                         RobotTask(wait=ROBOT_LOAD_TIME, load_store=pos_pickup.get_store_object(floor_plan)),
                         RobotTask(floor_plan=floor_plan, goto_pos=self.default_pos)]

    def is_idle(self):
        return len(self.task_list)==0

    def _update_rot_mat(self):
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
        self._update_rot_mat()

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