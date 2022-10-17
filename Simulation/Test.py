import cv2 as cv
import random

from RollContainer import RollContainer
from FloorPlan import FloorPlan, TIME_STEP_S, TIME_ROLL_CONTAINER_LOAD

def main():
    fp = FloorPlan(8)
    fp.header_text = "time="
    fp.draw(draw_circulation=False, draw_grid=True)
    fp.imshow("Test")
    cv.waitKey(0)

    rc_list = []
    for t in range(10):
        rr = random.randint(0,255)
        gg = random.randint(0,255)
        bb = random.randint(0,255)
        rc_list.append( RollContainer(0., 0., 3, (rr, gg, bb), t))

    path1 = fp.get_shortest_path(2, 0, parking1=0, buffer_lane2=1)
    path2 = fp.get_shortest_path(0, 3, buffer_lane1=1, store2=0, row2=5)
    path3 = fp.get_shortest_path(3, 1, store1=0, row1=5, parking2=2)

    t2 = -1
    t3 = -1
    t4 = -1
    t_load = 0.
    for t in range(600):
        sec = int(t*TIME_STEP_S) % 60
        mi  = int(t*TIME_STEP_S) // 60
        fp.header_text = f"time={mi:2d}:{sec:2d}"
        if t==50:   fp.robots[2].set_path(path1)
        if t>50 and t2<0 and fp.robots[2].is_at_end_point():
            t2 = t
            fp.robots[2].load_roll_container(fp.buffer_lanes[(0,1)].pop_roll_container())
            fp.robots[2].set_path(path2)

            fp.robots[1].set_path(fp.get_shortest_path(0, 0, parking1=0, buffer_lane2=1))

        if t3<0 and fp.robots[2].is_at_end_point():
            t3 = t
            fp.buffer_stores[(3, 0)].store_roll_container(5, fp.robots[2].unload_roll_container())
            fp.robots[2].set_path(path3)

        if t4<0 and fp.robots[1].is_at_end_point():
            t4 = t
            fp.robots[1].load_roll_container(fp.buffer_lanes[(0,1)].pop_roll_container())
            fp.robots[1].set_path(fp.get_shortest_path(0, 2, buffer_lane1=1, buffer_lane2=3))

        if t4>0 and fp.robots[1].is_at_end_point() and not fp.robots[1].rol is None:
            fp.buffer_lanes[(2,3)].store_roll_container( fp.robots[1].unload_roll_container())
            fp.robots[1].set_path(fp.get_shortest_path(2, 3, buffer_lane1=3, parking2=2))

        t_load += TIME_STEP_S
        if t_load>0.:
            if len(rc_list)>0:
                fp.buffer_lanes[(0, 1)].store_roll_container(rc_list.pop())
            t_load -= TIME_ROLL_CONTAINER_LOAD

        fp.step()
        fp.draw(draw_grid=True)
        fp.imshow("Test")
        cv.waitKey(40)

    cv.waitKey(0)
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()