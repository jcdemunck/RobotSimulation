import cv2 as cv
from FloorPlan import get_orientation, get_distance_city_block, get_path_len, \
                      TIME_STEP_S, W_FLOOR, H_FLOOR, \
                      ROBOT_SPEED, ROBOT_LOAD_TIME, ROBOT_UNLOAD_TIME, W1_ROBOT, W2_ROBOT, H1_ROBOT, H2_ROBOT



class RobotTask:
    def __init__(self, wait=0., floor_plan=None, coords_from=None, goto_pos=None, load_store=None, unload_store=None):
        self.task_type = "no_task"
        self.finished  = False

        if not floor_plan is None:
            if not coords_from is None and not goto_pos is None:
                self.path    = floor_plan.grid_graph.get_shortest_path(coords_from, goto_pos.get_coords())
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

                if(len(self.path))<=1:
                    self.finished = True

        elif not load_store is None:
            self.store     = load_store
            self.task_type = "pickup"

        elif not unload_store is None:
            self.store     = unload_store
            self.task_type = "unload"

        else:
            self.wait      = wait
            self.task_type = "wait"
            if self.wait<0:
                self.finished = True

    def time_step(self, robot):
        if self.task_type[0:4]=="goto":
            if self.segment>=len(self.path)-1:
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
                    self.offset = 0.
                    step_todo -= d
                    p_begin = p_end
                    self.segment += 1

            self.segment = len(self.path)  # use this to test whether robot is at end of path
            self.offset  = 0.
            robot.set_coords(self.path[-1])
            self.finished = True

        elif self.task_type=="pickup":
            robot.load_roll_container(self.store.pickup_roll_container())
            self.finished = True

        elif self.task_type=="unload":
            self.store.store_roll_container(robot.unload_roll_container())
            self.finished = True

        elif self.task_type=="wait":
            self.wait -= TIME_STEP_S
            if self.wait<=0.:
                self.finished = True

    def get_time_to_finish(self):
        if self.finished:
            return 0.

        if self.task_type[0:4]=="goto":
            p1 = self.path[self.segment  ]
            p2 = self.path[self.segment+1]
            distance_todo = get_distance_city_block(p1, p2) - self.offset
            if self.segment+2<len(self.path):
                p_old = p2
                for p in self.path[self.segment+2:-1]:
                    distance_todo += get_distance_city_block(p_old, p)
                    p_old = p
            return distance_todo/ROBOT_SPEED

        return TIME_STEP_S

class Robot:
    def __init__(self, w, h, orientation):
        self.w   = w
        self.h   = h
        self.o   = orientation
        self.rot = [[1, 0], [0, 1]]
        self.col = (200, 0, 0)
        self.rol = None

        self._update_rot_mat()

        self.task_list = []

    def set_coords(self, pos):
        self.w, self.h = pos
        if self.rol:
            self.rol.w, self.rol.h = pos
            self.rol.o = self.o

    def get_coords_offset(self, coords, s):
        if self.o==0 or self.o==2:    return coords[0] + (s if self.o==0 else -s), coords[1]
        if self.o==1 or self.o==3:    return coords[0], coords[1] + (s if self.o==1 else -s)

    def time_step(self):
        if len(self.task_list)<=0: return

        task = self.task_list[0]
        if task.finished:
            self.task_list.pop(0)
            if len(self.task_list)<=0: return
            task = self.task_list[0]

        task.time_step(self)
        if task.finished:
            self.task_list.pop(0)

    def get_time_to_pos(self, floor_plan, pos):
        coords1  = (self.w, self.h)
        coords2  = pos.get_coords()
        path     = floor_plan.grid_graph.get_shortest_path(coords1, coords2, get_distance_city_block)

        return get_path_len(path)/ROBOT_SPEED

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

    def wait_goto_load_store_park(self, floor_plan, wait, pos_load, pos_unload, pos_park):
        if not self.rol is None:
            print("ERROR: RollContainer.goto_load_store_park(). roll container already loaded")
            return

        self.task_list = []
        self.task_list.append(RobotTask(wait=wait))
        self.task_list.append(RobotTask(floor_plan=floor_plan, coords_from=(self.w, self.h), goto_pos=pos_load))
        self.task_list.append(RobotTask(load_store=pos_load.get_store_object(floor_plan)))
        self.task_list.append(RobotTask(wait=ROBOT_LOAD_TIME))
        self.task_list.append(RobotTask(floor_plan=floor_plan, coords_from=pos_load.get_coords(), goto_pos=pos_unload))
        self.task_list.append(RobotTask(unload_store=pos_unload.get_store_object(floor_plan)))
        self.task_list.append(RobotTask(wait=ROBOT_UNLOAD_TIME))
        self.task_list.append(RobotTask(floor_plan=floor_plan, coords_from=pos_unload.get_coords(), goto_pos=pos_park))

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
