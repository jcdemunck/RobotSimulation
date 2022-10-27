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


def main():
    fp = FloorPlan(8)
    for dock in range(N_DOCK):
        fp.docks[dock].set_color(destination_color_dict[DESTINATIONS[dock]])

    fp.header_text = "time="
    fp.draw(draw_circulation=False, draw_grid=True)
    fp.imshow("Test")
    cv.setWindowTitle("Test", "X-dock")
    cv.waitKey(0)

    random.seed(13)
    truck = create_truck(25, "Amsterdam", 5., 500.)

    wait_time = 0
    for t in range(1000):
        sec = int(t*TIME_STEP_S) % 60
        mi  = int(t*TIME_STEP_S) // 60
        fp.header_text = f"time={mi:2d}:{sec:02d}"
        if t==10:
            fp.start_unloading_truck(0, truck)
            wait_time = 0.

        if t>10 and t%10==0:
            # Make plan to process
            rc_incoming = fp.get_incoming_roll_containers(0)
            rc_incoming = [rio for rio in rc_incoming if not rio.roll_container.scheduled]
            robots      = sorted(fp.robots, key=lambda rob: rob.get_task_list_length())

            for robot, roll_io in zip(robots, rc_incoming):
                pos_pickup = Position(fp, 0, buffer_lane=roll_io.lane)

                if robot.is_idle():
                    wait  = max(0., roll_io.eta-robot.get_time_to_pos(fp, pos_pickup) + wait_time)
                    robot.wait_goto_pickup(fp, wait, pos_pickup)
                else:
                    robot.append_goto_pickup(fp, pos_pickup)

                roll_io.roll_container.scheduled = True
                wait_time += TIME_LOAD_TOTAL

        fp.time_step()
        fp.draw(draw_grid=True)
        fp.imshow("Test")
        cv.setWindowTitle("Test", "X-dock")
        k = cv.waitKey(50)
        if k&0xFF==ord(" "):
            cv.waitKey(0)

    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()