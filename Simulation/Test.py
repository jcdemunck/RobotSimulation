import cv2 as cv
import numpy as np
from pathlib import Path

from FloorPlan import FloorPlan
from Position import Position

from XdockParams import TIME_STEP_S, TIME_LOAD_BUFFER_LANE
from SimulationConfig import set_dock_names_colors, get_output_dock
from TruckPlan import TruckPlan
from Robot import BSM

from ModelParameters import ModelParams
M = ModelParams()
M.set_n_buffer_store(3)
M.set_n_buffer_row_col(8,3)
M.set_n_robot(32)
M.set_max_buffer_lane_store(18)


SIMULATE        = False

N_VIDEO_FRAME   = -1
video_out       = None
im_list         = []
DIR_VIDEO       = "C:/Users/MunckJande/OneDrive - PostNL/Documenten/Projecten/Robots_at_Xdocks/Video/"

TIME_LOAD_TOTAL = 1.5 * TIME_LOAD_BUFFER_LANE


def main():
    fp = FloorPlan()
    set_dock_names_colors(fp)

    if N_VIDEO_FRAME>0:
        frame_size = (fp.fig_width, fp.fig_height)
        video_out  = cv.VideoWriter("test0.avi", cv.VideoWriter_fourcc(*"MJPG"), 20., frame_size)

    fp.draw(draw_circulation=True, draw_grid=True)

    fp.imshow("Test")
    cv.setWindowTitle("Test", "X-dock")
    cv.moveWindow("Test", 10, 10)
    cv.waitKey(0)

    P = TruckPlan(SIMULATE)
    truck_list  = [t for t in P.truck_list]
    samp_start  = int(P.start_time/TIME_STEP_S)
    samp_end    = int(P.end_time  /TIME_STEP_S)

    fp.time_sec = samp_start*TIME_STEP_S
    fp.set_truck_list(truck_list)

    draw_step  = 1
    show_delay = 1
    for sample in range(samp_start, samp_end):

        for dock in range(M.N_DOCK):
            rc_incoming = fp.get_incoming_roll_containers(dock)
            truck       = fp.docks[dock].truck

            if len(rc_incoming)>0:
                priority = 0 if truck is None or not truck.inbound else fp.get_nrc_incoming(dock)

                # assign robots to incoming roll containers, until all roll containers are assigned to robot
                rob_list = sorted(fp.robots, key=lambda rob: rob.get_task_list_length())

                for n, (robot, roll_io) in enumerate(zip(rob_list, rc_incoming), start=1):
                    pos_pickup = Position(fp, dock, buffer_lane=roll_io.lane)

                    if robot.is_idle():
                        wait  = max(0., roll_io.eta + n*TIME_LOAD_TOTAL -robot.get_time_to_pos(fp, pos_pickup) )
                        robot.wait_process_incoming(fp, wait, pos_pickup)
                    else:
                        robot.append_process_incoming(fp, pos_pickup, priority>3)

                    roll_io.roll_container.scheduled = True

            if truck is None: continue

            # Process outbound trucks
            if not truck.inbound:
                # Get all buffers with required destination, but skip stores that are unused.
                buffer_list = BSM.get_buffer_list(truck.destination, truck.prios[0])
                buffer_list = [(d,b) for (d,b) in buffer_list if not fp.buffer_stores[d,b].is_store_unused()]
                dock_dest   = get_output_dock(truck.destination, truck.prios[0])
                rob_list    = BSM.get_sorted_robots(fp.robots, dock_dest)
                r           = 0
                for (dock_orig,store) in buffer_list: # plan robots from buffer store to output lane
                    row     = fp.buffer_stores[dock_orig, store].get_row_not_scheduled()
                    lane    = fp.get_best_available_lane(dock_dest, output=True)

                    while lane>=0 and row>=0:
                        fp.buffer_stores[dock_orig, store].schedule_roll_container(row)
                        fp.buffer_lanes[dock_dest, lane].reserve_store()

                        pos_pickup = Position(fp, dock_orig, buffer_store=store, row=row, col=0)
                        pos_unload = Position(fp, dock_dest, buffer_lane=lane)
                        rob = rob_list[r]
                        rob.insert_process_store(fp, pos_pickup, pos_unload)

                        r    = (r+1)%len(rob_list)
                        row  = fp.buffer_stores[dock_orig, store].get_row_not_scheduled()
                        lane = fp.get_best_available_lane(dock_dest, output=True)

        fp.time_step()
        if not ((sample-samp_start+1)%draw_step) or sample+1==samp_end:
            fp.draw()
            fp.imshow("Test")
            cv.setWindowTitle("Test", "X-dock")
            k = cv.waitKey(show_delay)
            if k&0xFF==ord(' '):
                cv.waitKey(0)
            elif k&0xFF==ord('+'):
                if show_delay==1:
                    draw_step = int(2*draw_step)
                else:
                    show_delay = max(1, int(show_delay/2))
            elif k&0xFF==ord('-'):
                if draw_step==1:
                    show_delay = int(2*show_delay)
                else:
                    draw_step = max(1, int(draw_step/2))
            elif k&0xFF==ord('q'):
                break

        if N_VIDEO_FRAME>0:
            frame = sample-samp_start
            if frame<N_VIDEO_FRAME:
                im_list.append(np.copy(fp.figure))
            if frame==N_VIDEO_FRAME or sample==samp_end:
                for im in im_list:
                    video_out.write(im)
                video_out.release()
                Path(DIR_VIDEO+"test0.avi").unlink(missing_ok=True)
                Path("test0.avi").rename(DIR_VIDEO+"test0.avi")
                break

    fp.log_results()
    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()