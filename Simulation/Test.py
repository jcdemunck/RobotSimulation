import cv2 as cv
import numpy as np
from pathlib import Path

from FloorPlan import FloorPlan
from Position import Position

from XdockParams import TIME_STEP_S, N_DOCK, TIME_ROLL_CONTAINER_LOAD, N_BUFFER_STORE
from SimulationConfig import set_dock_names_colors
from Truck import get_truck_list, trucks_from_file


TIME_LOAD_TOTAL = 1.5*TIME_ROLL_CONTAINER_LOAD

N_VIDEO_FRAME  = -1
video_out      = None
im_list        = []
DIR_VIDEO      = "C:/Users/MunckJande/OneDrive - PostNL/Documenten/Projecten/Robots_at_Xdocks/Video/"

class ProcessTruckLoad:
    def __init__(self, dock, n_roll_containers):
        self.wait_time        =  0.
        self.dock             =  dock
        self.nrc_not_assigned =  n_roll_containers
        self.out_going_truck  =  n_roll_containers==0

def main():
    fp = FloorPlan(2*N_DOCK)
    set_dock_names_colors(fp)

    if N_VIDEO_FRAME>0:
        frame_size = (fp.fig_width, fp.fig_height)
        video_out  = cv.VideoWriter("test0.avi", cv.VideoWriter_fourcc(*"MJPG"), 20., frame_size)

    fp.header_text = "time="
    fp.draw(draw_circulation=True, draw_grid=False)

    fp.imshow("Test")
    cv.setWindowTitle("Test", "X-dock")
    cv.moveWindow("Test", 10, 10)
    cv.waitKey(0)



    truck_list = get_truck_list() #trucks_from_file()  #
    samp_start = int( truck_list[ 0].arrival           /TIME_STEP_S)
    samp_end   = int((truck_list[-1].departure + 1000.)/TIME_STEP_S)

    truck_process_list = []
    for sample in range(samp_start, samp_end):
        time_sec       = sample*TIME_STEP_S
        fp.header_text = f"t={int(time_sec)//3600:2d}:{(int(time_sec)//60)%60:2d}:{int(time_sec)%60:02d}"

        if len(truck_list)>0 and time_sec>truck_list[0].arrival:
            truck = truck_list.pop(0)
            dock  = truck.dock
            truck_process_list.append(ProcessTruckLoad(dock, len(truck.truck_load)))
            if truck.destination:
                fp.start_loading_truck(dock, truck)
            else:
                fp.start_unloading_truck(dock, truck)


        for proc in truck_process_list:
            dock = proc.dock
            robots = sorted(fp.robots, key=lambda rob: rob.get_task_list_length())
            if not proc.out_going_truck:
                # assign robots to incoming roll containers, until all roll containers are assigned to robot
                rc_incoming = fp.get_incoming_roll_containers(dock)
                rc_incoming = [rio for rio in rc_incoming if not rio.roll_container.scheduled]

                for robot, roll_io in zip(robots, rc_incoming):
                    pos_pickup = Position(fp, dock, buffer_lane=roll_io.lane)

                    if robot.is_idle():
                        wait  = max(0., roll_io.eta-robot.get_time_to_pos(fp, pos_pickup) + proc.wait_time)
                        robot.wait_process_incoming(fp, wait, pos_pickup)
                    else:
                        robot.append_process_incoming(fp, pos_pickup)

                    roll_io.roll_container.scheduled = True
                    proc.nrc_not_assigned           -= 1
                    proc.wait_time                  += TIME_LOAD_TOTAL

            else:
                for store in range(N_BUFFER_STORE):
                    row  = fp.buffer_stores[(dock, store)].get_row_not_scheduled()
                    lane = fp.get_available_output_lane(dock)
                    if 0<=row<len(robots) and lane>0:
                        fp.buffer_stores[(dock, store)].schedule_roll_container(row)
                        fp.buffer_lanes[(dock, lane)].reserve_store()

                        pos_pickup = Position(fp, dock, buffer_store=store, row=row, col=0)
                        pos_unload = Position(fp, dock, buffer_lane=lane)
                        robots[row].insert_process_store(fp, pos_pickup, pos_unload)


            truck_process_list = [proc for proc in truck_process_list if proc.nrc_not_assigned>0 or proc.out_going_truck]


        fp.time_step()
        fp.draw(draw_grid=True)
        fp.imshow("Test")
        cv.setWindowTitle("Test", "X-dock")
        k = cv.waitKey(1)
        if k&0xFF==ord(' '):
            cv.waitKey(0)
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