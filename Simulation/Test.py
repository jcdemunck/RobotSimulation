import cv2 as cv
import random

from RollContainer import RollContainer
from Position import Position
from Truck import Truck
from FloorPlan import FloorPlan, TIME_STEP_S, N_DOCK, TIME_ROLL_CONTAINER_LOAD,\
                      DESTINATIONS, destination_color_dict



TIME_LOAD_TOTAL = 1.5*TIME_ROLL_CONTAINER_LOAD

def create_truck(nrc, destination, t_arrive, t_departure):
    rc_list = []
    for t in range(nrc):
        dest  = DESTINATIONS[random.randint(0, len(DESTINATIONS)-1)]
        shift = random.randint(0,9)
        rc_list.append( RollContainer(0., 0., 3, dest, shift, destination_color_dict[dest]))

    return Truck(rc_list, destination, destination_color_dict[destination], t_arrive, t_departure)

class ProcessTruck:
    def __init__(self, dock, n_roll_containers):
        self.wait_time              =  0.
        self.dock                   =  dock
        self.n_roll_containers_left =  n_roll_containers


def main():
    fp = FloorPlan(8)
    for dock in range(N_DOCK):
        fp.docks[dock].set_color(destination_color_dict[DESTINATIONS[dock]])

    fp.header_text = "time="
    fp.draw(draw_circulation=False, draw_grid=True)
    fp.imshow("Test")
    cv.setWindowTitle("Test", "X-dock")
    cv.moveWindow("Test", 10, 10)
    cv.waitKey(0)

    random.seed(13)
    truck_list = sorted([create_truck(25, "Amsterdam", 5., 500.),
                  create_truck(30, "Assen", 80., 500.),
                  create_truck(20, "Groningen", 150., 500.)], key=lambda tr: tr.arrival)


    truck_process_list = []
    for sample in range(2500):
        time_sec       = sample*TIME_STEP_S
        fp.header_text = f"time={int(time_sec) // 60:2d}:{int(time_sec) % 60:02d}"
        if len(truck_list)>0 and time_sec>truck_list[0].arrival:
            truck = truck_list.pop(0)
            dock  = DESTINATIONS.index(truck.destination)
            truck_process_list.append(ProcessTruck(dock, len(truck.truck_load)))
            fp.start_unloading_truck(dock, truck)

        if len(truck_process_list):
            # Make plan to process
            for proc in truck_process_list:
                rc_incoming = fp.get_incoming_roll_containers(proc.dock)
                rc_incoming = [rio for rio in rc_incoming if not rio.roll_container.scheduled]
                robots      = sorted(fp.robots, key=lambda rob: rob.get_task_list_length())

                for robot, roll_io in zip(robots, rc_incoming):
                    pos_pickup = Position(fp, proc.dock, buffer_lane=roll_io.lane)

                    if robot.is_idle():
                        wait  = max(0., roll_io.eta-robot.get_time_to_pos(fp, pos_pickup) + proc.wait_time)
                        robot.wait_goto_pickup(fp, wait, pos_pickup)
                    else:
                        robot.append_goto_pickup(fp, pos_pickup)

                    roll_io.roll_container.scheduled = True
                    proc.n_roll_containers_left     -= 1
                    proc.wait_time                  += TIME_LOAD_TOTAL

            truck_process_list = [proc for proc in truck_process_list if proc.n_roll_containers_left>0]


        fp.time_step()
        fp.draw(draw_grid=True)
        fp.imshow("Test")
        cv.setWindowTitle("Test", "X-dock")
        k = cv.waitKey(50)
        if k&0xFF==ord(' '):
            cv.waitKey(0)
        elif k&0xFF==ord('q'):
            break

    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()