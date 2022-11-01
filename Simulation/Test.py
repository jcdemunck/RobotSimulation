import cv2 as cv


from FloorPlan import FloorPlan
from Position import Position

from XdockParams import TIME_STEP_S, N_DOCK, TIME_ROLL_CONTAINER_LOAD
from SimulationConfig import DESTINATIONS, destination_color_dict, \
                             assign_docks, get_truck_list, trucks_from_file


TIME_LOAD_TOTAL = 1.5*TIME_ROLL_CONTAINER_LOAD


class ProcessTruckLoad:
    def __init__(self, dock, n_roll_containers):
        self.wait_time        =  0.
        self.dock             =  dock
        self.nrc_not_assigned =  n_roll_containers
        self.out_going_truck  =  n_roll_containers==0

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

    truck_list = trucks_from_file()#get_truck_list()

    assign_docks(truck_list)

    truck_process_list = []
    for sample in range(2500):
        time_sec       = sample*TIME_STEP_S
        fp.header_text = f"time={int(time_sec) // 60:2d}:{int(time_sec) % 60:02d}"
        if len(truck_list)>0 and time_sec>truck_list[0].arrival:
            truck = truck_list.pop(0)
            dock  = truck.dock
            truck_process_list.append(ProcessTruckLoad(dock, len(truck.truck_load)))
            if truck.destination:
                fp.start_loading_truck(dock, truck)
            else:
                fp.start_unloading_truck(dock, truck)

        if len(truck_process_list):
            # Make plan to process
            for proc in truck_process_list:
                if proc.out_going_truck: continue

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
                    proc.nrc_not_assigned           -= 1
                    proc.wait_time                  += TIME_LOAD_TOTAL

            truck_process_list = [proc for proc in truck_process_list if proc.nrc_not_assigned>0 or proc.out_going_truck]


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