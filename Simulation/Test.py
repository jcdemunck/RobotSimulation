import cv2 as cv
import numpy as np
from pathlib import Path

from FloorPlan import FloorPlan
from Position import Position

from XdockParams import TIME_STEP_S, N_DOCK, TIME_LOAD_BUFFER_LANE
from SimulationConfig import set_dock_names_colors, get_output_dock
from TruckPlan import TruckPlan
from Truck import Truck
from Robot import BSM



SIMULATE        = False
TIME_LOAD_TOTAL = 1.5 * TIME_LOAD_BUFFER_LANE

N_VIDEO_FRAME  = -1
video_out      = None
im_list        = []
DIR_VIDEO      = "C:/Users/MunckJande/OneDrive - PostNL/Documenten/Projecten/Robots_at_Xdocks/Video/"


class TruckProcessed(Truck):
    def __init__(self, truck):
        Truck.__init__(self, truck.arrival, truck.departure, truck.color, truck.destination, truck.prios, truck.truck_load, truck.ID)
        self.dock             = truck.dock
        self.wait_time        = 0.
        self.nrc_not_assigned = len(truck.truck_load)

def main():
    fp = FloorPlan(2*N_DOCK)
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

    truck_process_list = []
    draw_step  = 1
    show_delay = 1
    for sample in range(samp_start, samp_end):
        time_sec = sample*TIME_STEP_S

        if len(truck_list)>0 and time_sec>truck_list[0].arrival:
            truck = truck_list.pop(0)
            truck_process_list.append(TruckProcessed(truck))

        for proc in truck_process_list:
            dock   = proc.dock
            robots = sorted(fp.robots, key=lambda rob: rob.get_task_list_length())
            if proc.inbound:
                # assign robots to incoming roll containers, until all roll containers are assigned to robot
                rc_incoming = fp.get_incoming_roll_containers(dock)
                rc_incoming = [rio for rio in rc_incoming if not rio.roll_container.scheduled]

                for robot, roll_io in zip(robots, rc_incoming):
                    pos_pickup = Position(fp, dock, buffer_lane=roll_io.lane)

                    if robot.is_idle():
                        wait  = max(0., roll_io.eta + proc.wait_time -robot.get_time_to_pos(fp, pos_pickup) )
                        robot.wait_process_incoming(fp, wait, pos_pickup)
                    else:
                        robot.append_process_incoming(fp, pos_pickup)

                    roll_io.roll_container.scheduled = True
                    proc.nrc_not_assigned           -= 1
                    proc.wait_time                  += TIME_LOAD_TOTAL

            else:
                buffer_list = BSM.get_buffer_list(proc.destination, proc.prios[0])
                buffer_list = [(d, b) for (d,b) in buffer_list if not fp.buffer_stores[d, b].is_store_unused()]

                dock_dest   = get_output_dock(proc.destination, proc.prios[0])
                rob_list    = BSM.get_sorted_robots(fp.robots, dock_dest)
                r           = 0
                for (dock,store) in buffer_list: # plan robots from buffer store to output lane
                    row       = fp.buffer_stores[dock, store].get_row_not_scheduled()
                    lane      = fp.get_best_available_lane(dock_dest, output=True)

                    while lane>=0 and row>=0:
                        fp.buffer_stores[dock, store].schedule_roll_container(row)
                        fp.buffer_lanes[dock_dest, lane].reserve_store()

                        pos_pickup = Position(fp, dock, buffer_store=store, row=row, col=0)
                        pos_unload = Position(fp, dock_dest, buffer_lane=lane)
                        rob = rob_list[r]
                        rob.insert_process_store(fp, pos_pickup, pos_unload)

                        r    = (r+1)%len(rob_list)
                        row  = fp.buffer_stores[dock, store].get_row_not_scheduled()
                        lane = fp.get_best_available_lane(dock_dest, output=True)

            truck_process_list = [proc for proc in truck_process_list if proc.nrc_not_assigned>0 or not proc.inbound]

        fp.time_step()
        if not ((sample-samp_start+1)%draw_step):
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
    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()