import cv2 as cv
import random

from RollContainer import RollContainer
from Position import Position
from FloorPlan import FloorPlan, TIME_STEP_S, N_DOCK, TIME_ROLL_CONTAINER_LOAD


DESTINATIONS            = ["Amsterdam","Apeldoorn","Groningen","Assen"]
destination_color_dict  = {"Amsterdam": (0,255,200), "Apeldoorn": (255,200,0), "Groningen":(100,100,255), "Assen":(100,0,50)}

TIME_LOAD_TOTAL = 1.5*TIME_ROLL_CONTAINER_LOAD

def main():
    fp = FloorPlan(8)
    for dock in range(N_DOCK):
        fp.docks[dock].set_color(destination_color_dict[DESTINATIONS[dock]])

    fp.header_text = "time="
    fp.draw(draw_circulation=False, draw_grid=True)
    fp.imshow("Test")
    cv.waitKey(0)

    random.seed(13)
    rc_list = []
    for t in range(20):
        dest  = DESTINATIONS[random.randint(0, len(DESTINATIONS)-1)]
        shift = random.randint(1,6)
        rc_list.append( RollContainer(0., 0., 3, dest, shift, destination_color_dict[dest]))

    wait_time = 0
    for t in range(1000):
        sec = int(t*TIME_STEP_S) % 60
        mi  = int(t*TIME_STEP_S) // 60
        fp.header_text = f"time={mi:2d}:{sec:02d}"
        if t==10:
            fp.start_unloading_truck(0, rc_list)
            wait_time = 0.

        if t>10 and t%10==0:
            # Make plan to process
            rc_incoming = fp.get_incoming_roll_containers(0)
            rc_incoming = [rio for rio in rc_incoming if not rio.roll_container.scheduled]
            robots      = fp.get_robots_idle()

            for r, (rob, roll_io) in enumerate(zip(robots, rc_incoming)):
                pos_park   = Position(fp, r%N_DOCK, parking=r//N_DOCK)
                pos_pickup = Position(fp, 0, buffer_lane=roll_io.lane)
                wait  = max(0., roll_io.eta-rob.get_time_to_pos(fp, pos_pickup) + wait_time)
                shift = roll_io.roll_container.shift
                dest  = roll_io.roll_container.dest
                if shift>5:
                    pos_store = Position(fp, DESTINATIONS.index(dest), buffer_store=0, row=shift-5, col=0)
                else:
                    pos_store = Position(fp, DESTINATIONS.index(dest), buffer_lane=3)
                rob.wait_goto_load_store_park(fp, wait, pos_pickup, pos_store, pos_park)

                roll_io.roll_container.scheduled = True
                wait_time += TIME_LOAD_TOTAL

        fp.time_step()
        fp.draw(draw_grid=True)
        fp.imshow("Test")
        cv.waitKey(40)

    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()