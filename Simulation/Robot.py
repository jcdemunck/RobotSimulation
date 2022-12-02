from collections import defaultdict, Counter
import cv2 as cv


from XdockParams import TIME_STEP_S,  \
                        ROBOT_SPEED, ROBOT_LOAD_TIME, ROBOT_UNLOAD_TIME, W1_ROBOT, W2_ROBOT, H1_ROBOT, H2_ROBOT, \
                        LOG_INTERVAL_ROBOT,\
                        get_orientation, get_distance_city_block, get_path_len
from ModelParameters import ModelParams as M
from ModelParameters import get_log_filename

from BufferStoreManager import BufferStoreManager


BSM = BufferStoreManager()

def print_tasks(task_list):
    for t,task in enumerate(task_list):
        print(t, task.task_type)

class RobotTask:
    log_tasks = ["park", "dummy", "goto_buffer_lane", "goto_buffer_store", "goto_parking", "goto",
                 "pickup_lane", "pickup_store", "unload", "wait", "begin_task", "end_task", "find_destination"]
    lastID    = 0
    def __init__(self, wait=-1., floor_plan=None, goto_pos=None, load_lane=None, load_store=None, unload=None, begin=None, find_destination=False):

        # Set default values
        self.ID           = RobotTask.lastID
        RobotTask.lastID += 1
        self.task_type    = "dummy"
        self.finished     = True
        self.path         = []
        self.goto         = (0.,0.)
        self.segment      = 0
        self.offset       = 0.
        self.wait         = 0.
        self.store        = None

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

        elif not load_lane is None:
            self.wait      = wait
            self.store     = load_lane
            self.task_type = "pickup_lane"
            self.finished  = False

        elif not load_store is None:
            self.wait      = wait
            self.store     = load_store
            self.task_type = "pickup_store"
            self.finished  = False

        elif not unload is None:
            self.wait      = wait
            self.store     = unload
            self.task_type = "unload"
            self.finished  = False

        elif wait>=0.:
            self.wait      = wait
            self.task_type = "wait"
            self.finished  = self.wait<=0.

        elif not begin is None:
            self.task_type = "begin_task" if begin is True else "end_task"

        elif find_destination is True:
            self.task_type = "find_destination"
            self.finished  = False

    def __str__(self):
        text  = f"ID = {self.ID:d} \n"
        text += f"task = {self.task_type:s} \n"
        text += f"finished = {str(self.finished):s} \n"
        text += f"segment = {self.segment:d}\n"
        text += f"offset = {self.offset:f}\n"
        if len(self.path)>0: text+=f"p0={str(self.path[0]):s}\n"
        text += f"goto = {str(self.goto):s}\n"
        text += f"wait = {self.wait:f}\n"
        return text

    def _is_dummy_task(self):
        return self.task_type in ["dummy","begin_task","end_task"]

    def time_step(self, robot, floor_plan):
        if self.task_type[0:4]=="goto":
            if len(self.path)==0:
                self.path = floor_plan.grid_graph.get_shortest_path((robot.w,robot.h), self.goto)

                if len(self.path)==0:
                    print("ERROR: Robot.time_step(). Path not found.")
                    print(robot.ID, (robot.w,robot.h), (robot.w,robot.h) in floor_plan.grid_graph.v_neighbors)
                    print(self)

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

        elif self.task_type=="pickup_lane" or self.task_type=="pickup_store":
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
            if self.wait<=0.:
                self.finished = True
            else:
                self.wait -= TIME_STEP_S

        elif self.task_type=="find_destination":
            # Pickup task finished. Insert goto.
                robot._complete_process_incoming(floor_plan)
                self.finished = True

    def get_time_to_finish(self):
        if self.finished:
            return 0.
        if self.task_type[0:4]=="goto":
            return M.W_FLOOR/(2*ROBOT_SPEED)
        if self.task_type=="wait":
            return self.wait
        return TIME_STEP_S

from functools import update_wrapper, partial
class RobotLogger:
    def __init__(self, func):
        update_wrapper(self, func)
        self.func = func

        self.time      = 0.
        self.samp      = 0
        self.task_dict = Counter()            # current number of tasks of each robot
        self.rob_dict  = defaultdict(Counter) # number of times each task type is executed, for each robot
        self.log_file  = ""

    def __get__(self, obj, objtype):
        return partial(self.__call__, obj)

    def __call__(self, rob, *args, **kwargs):
        # Write log file header
        if self.log_file=="":
            self.log_file = get_log_filename("Robots")
            with open(self.log_file, "w") as fp:
                fp.write("ID\ttime\t" + '\t'.join(RobotTask.log_tasks) + "\tn_tasks\n")

        # Add current status to log file
        if rob.ID==0:
            if self.samp==0:
                with open(self.log_file, "a") as fp:
                    for id in sorted(self.rob_dict):
                        times = [self.rob_dict[id][ta] * TIME_STEP_S/3600. for ta in RobotTask.log_tasks]
                        n_log = int(0.5+LOG_INTERVAL_ROBOT/TIME_STEP_S)
                        if id>=0:
                            n_tasks = self.task_dict[id]/n_log
                            line    = f"{str(id) :s}\t{self.time/3600.:9.3f}\t" + '\t'.join(f"{t:8.1f}" for t in times) + '\t'+f"{n_tasks:8.1f}"+'\n'
                        else:
                            n_rob   = len(args[0].robots)
                            times   = [t/n_rob for t in times]  # Compute mean time
                            n_tasks = sum(v for v in self.task_dict.values())/(n_rob*n_log)
                            line    = f"total\t{self.time/3600.:9.3f}\t" + '\t'.join(f"{t:8.1f}" for t in times) + '\t'+f"{n_tasks:8.1f}"+'\n'
                        fp.write(line)
                self.task_dict = Counter()

            # Timings
            self.time += TIME_STEP_S
            self.samp += 1
            if self.samp*TIME_STEP_S >= LOG_INTERVAL_ROBOT:
                self.samp = 0

        # Keep track of stats
        self.task_dict[rob.ID] += len(rob.task_list)
        task = "park" if len(rob.task_list)==0 else rob.task_list[0].task_type
        self.rob_dict[rob.ID][task] += 1
        self.rob_dict[-1    ][task] += 1

        return self.func(rob, *args, **kwargs)

class Robot:
    lastID = 0
    def __init__(self, w, h, parking_pos, color=None):
        self.w           = w
        self.h           = h
        self.o           = 3
        self.rot         = [[1, 0], [0, 1]]
        self.col         = (200, 0, 0) if color is None else color
        self.rol         = None
        self.default_pos = parking_pos
        self.ID          = Robot.lastID
        Robot.lastID    += 1
        self._update_rot_mat()

        self.task_list = []

    def __str__(self):
        text  = f"w = {self.w:7.2f}\n"
        text += f"h = {self.h:7.2f}\n"
        text += f"o = {self.o:d}\n"
        text += f"ID = {self.ID:d}\n"
        text += f"pos = {str(self.default_pos):s}\n"
        text += f"n_task = {len(self.task_list):d}\n"
        for task in self.task_list:
            text += "\t\t"+str(task).replace('\n','\t') +'\n'
        return text

    def set_coords(self, pos):
        self.w, self.h = pos
        if self.rol:
            self.rol.w, self.rol.h = pos
            self.rol.o = self.o

    def get_coords_offset(self, coords, s):
        if self.o==0 or self.o==2:    return coords[0] + (s if self.o==0 else -s), coords[1]
        if self.o==1 or self.o==3:    return coords[0], coords[1] + (s if self.o==1 else -s)

    @RobotLogger
    def time_step(self, floor_plan):
        if len(self.task_list)<=0: return

        task = self.task_list[0]
        while task.finished or task._is_dummy_task():
            self.task_list.pop(0)
            if len(self.task_list)<=0: return
            task = self.task_list[0]

        task.time_step(self, floor_plan)
        if task.finished:
            self.task_list.pop(0)

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

    def _append_parking(self, floor_plan):
        if self.task_list[-2].task_type!="goto_parking":
            self.task_list += [RobotTask(begin=True), RobotTask(floor_plan=floor_plan, goto_pos=self.default_pos), RobotTask(begin=False)]

    def wait_process_incoming(self, floor_plan, wait, pos_pickup):
        if not self.rol is None:
            print("ERROR: RollContainer.wait_process_incoming(). Roll container already loaded")
            return

        if len(self.task_list)>0:
            print("ERROR: RollContainer.wait_process_incoming(). Task list not empty.")
            return

        # new, incomplete task (storage destination not yet known)
        self.task_list = [RobotTask(begin=True),
                          RobotTask(wait=wait),
                          RobotTask(floor_plan=floor_plan, goto_pos=pos_pickup),
                          RobotTask(wait=ROBOT_LOAD_TIME, load_lane=pos_pickup.get_store_object(floor_plan)),
                          RobotTask(find_destination=True)]

    def append_process_incoming(self, floor_plan, pos_pickup, prepend=False):
        # new, incomplete task (storage destination not yet known)
        new_tasks = [RobotTask(begin=True),
                     RobotTask(floor_plan=floor_plan, goto_pos=pos_pickup),
                     RobotTask(wait=ROBOT_LOAD_TIME, load_lane=pos_pickup.get_store_object(floor_plan)),
                     RobotTask(find_destination=True)]

        n_tasks = len(self.task_list)
        if n_tasks==0:
            self.task_list = new_tasks

        # find last finished task
        else:
            end_list = -1
            if prepend: #put new task at the beginning of task list
                end_list = -1
                for t, task in enumerate(self.task_list):
                    if task.task_type=="end_task" or task.task_type=="find_destination":
                        end_list = t + 1
                        break

            else: # put new task at the end of list
                if n_tasks>2 and self.task_list[-2].task_type=="goto_parking":
                    for t,task in enumerate(reversed(self.task_list[:-2])):
                        if task.task_type=="end_task" or task.task_type=="find_destination":
                            end_list = n_tasks-2-t
                            break
                else:
                    for t,task in enumerate(reversed(self.task_list)):
                        if task.task_type=="end_task" or task.task_type=="find_destination":
                            end_list = n_tasks-t
                            break

            # insert new tasks
            self.task_list = self.task_list[:end_list]  + new_tasks + self.task_list[end_list:]

        self._append_parking(floor_plan)

    def _complete_process_incoming(self, floor_plan):
        if self.rol is None:
            print(self)
        pos_store = BSM.choose_and_reserve_store(floor_plan, self.rol)
        task_list = [RobotTask(floor_plan=floor_plan, goto_pos=pos_store),
                     RobotTask(wait=ROBOT_UNLOAD_TIME, unload=pos_store.get_store_object(floor_plan)),
                     RobotTask(begin=False)]

        # insert new tasks
        self.task_list = self.task_list[:1] + task_list + self.task_list[1:]
        self._append_parking(floor_plan)

    def insert_process_store(self, floor_plan, pos_pickup, pos_unload):

        new_tasks = [RobotTask(begin=True),
                     RobotTask(floor_plan=floor_plan, goto_pos=pos_pickup),
                     RobotTask(wait=ROBOT_LOAD_TIME, load_store=pos_pickup.get_store_object(floor_plan)),
                     RobotTask(floor_plan=floor_plan, goto_pos=pos_unload),
                     RobotTask(wait=ROBOT_UNLOAD_TIME, unload=pos_unload.get_store_object(floor_plan)),
                     RobotTask(begin=False)]

        if len(self.task_list)==0:
            self.task_list = new_tasks

        else:# Find first unfinished task
            end_list = -1
            for t,task in enumerate(self.task_list):
                if task.task_type=="end_task" or task.task_type=="find_destination":
                    end_list = t+1
                    break

        # insert new tasks
            self.task_list = self.task_list[:end_list] + new_tasks + self.task_list[end_list:]

        self._append_parking(floor_plan)

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

        self.w = max(0., min(M.W_FLOOR, self.w))
        self.h = max(0., min(M.H_FLOOR, self.h))
        if not self.rol is None:
            self.rol.w = self.w
            self.rol.h = self.h

    def rotate(self, left):
        step = 1 if left else 3
        self.o = (self.o + step) % 4
        self._update_rot_mat()

        if not self.rol is None:
            self.rol.rotate(left)

    def load_roll_container(self, rol):
        self.rol           = rol
        self.rol.w         = self.w
        self.rol.h         = self.h
        self.rol.o         = self.o
        self.rol.scheduled = False

    def unload_roll_container(self):
        if self.rol is None:
            print("ERROR: Robot.unload_roll_container(). Robot not loaded. \n", self)
        rol        = self.rol
        self.rol   = None
        return rol