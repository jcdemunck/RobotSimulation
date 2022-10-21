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

    n_robots_active = 8
    robots_planned  = False
    for t in range(600):
        sec = int(t*TIME_STEP_S) % 60
        mi  = int(t*TIME_STEP_S) // 60
        fp.header_text = f"time={mi:2d}:{sec:02d}"
        if t==10:
            fp.start_unloading_truck(0, rc_list)

        if t>10 and not robots_planned:
            # Make plan to process (first n_robots_active)
            rc_incoming = fp.get_incoming_roll_containers(0)
            if len(rc_incoming)>=n_robots_active:
                robots_planned = True
                for r in range(n_robots_active):
                    pos_park   = Position(fp, r%N_DOCK, parking=r//N_DOCK)
                    roll_io    = rc_incoming[r]
                    pos_pickup = Position(fp, 0, buffer_lane=roll_io.lane)
                    wait       = max(0., roll_io.eta-fp.robots[r].get_time_to_pos(fp, pos_pickup) + r*TIME_LOAD_TOTAL)
                    if roll_io[3]>5:
                        pos_store = Position(fp, DESTINATIONS.index(roll_io.destination), buffer_store=0, row=roll_io.shift-5, col=0)
                    else:
                        pos_store = Position(fp, DESTINATIONS.index(roll_io.destination), buffer_lane=3)
                    fp.robots[r].wait_goto_load_store_park(fp, wait, pos_pickup, pos_store, pos_park)

        fp.time_step()
        fp.draw(draw_grid=True)
        fp.imshow("Test")
        cv.waitKey(40)

    cv.waitKey(0)
    cv.destroyAllWindows()



    return
    pos1 = Position(fp, dock=2, parking=0)
    pos2 = Position(fp, dock=0, buffer_lane=1)
    pos3 = Position(fp, dock=3, buffer_store=0, row=5, col=0)
    pos4 = Position(fp, dock=1, parking=2)


    path1 = fp.get_shortest_path(pos1, pos2)
    path2 = fp.get_shortest_path(pos2, pos3)
    path3 = fp.get_shortest_path(pos3, pos4)




    t2 = -1
    t3 = -1
    t4 = -1
    for t in range(400):
        sec = int(t*TIME_STEP_S) % 60
        mi  = int(t*TIME_STEP_S) // 60
        fp.header_text = f"time={mi:2d}:{sec:02d}"
        if t==1:
            fp.start_unloading_truck(0, rc_list)

        if t==50:   fp.robots[2].set_path(path1)
        if t>50 and t2<0 and fp.robots[2].is_at_end_point():
            t2 = t
            fp.robots[2].load_roll_container(fp.buffer_lanes[(0,1)].pickup_roll_container())
            fp.robots[2].set_path(path2)

            pos1 = Position(fp, dock=0, parking=0)
            pos2 = Position(fp, dock=0, buffer_lane=1)
            fp.robots[1].set_path(fp.get_shortest_path(pos1, pos2))

        if t3<0 and fp.robots[2].is_at_end_point():
            t3 = t
            fp.buffer_stores[(3, 0)].store_roll_container(5, fp.robots[2].unload_roll_container())
            fp.robots[2].set_path(path3)

        if t4<0 and fp.robots[1].is_at_end_point():
            t4 = t
            fp.robots[1].load_roll_container(fp.buffer_lanes[(0,1)].pickup_roll_container())

            pos1 = Position(fp, dock=0, buffer_lane=1)
            pos2 = Position(fp, dock=2, buffer_lane=3)
            fp.robots[1].set_path(fp.get_shortest_path(pos1, pos2))

        if t4>0 and fp.robots[1].is_at_end_point() and not fp.robots[1].rol is None:
            fp.buffer_lanes[(2,3)].store_roll_container( fp.robots[1].unload_roll_container())

            pos1 = Position(fp, dock=2, buffer_lane=3)
            pos2 = Position(fp, dock=3, parking=2)

            fp.robots[1].set_path(fp.get_shortest_path(pos1, pos2))


        fp.time_step()
        fp.draw(draw_grid=True)
        fp.imshow("Test")
        cv.waitKey(40)

    cv.waitKey(0)
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()