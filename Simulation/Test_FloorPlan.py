import cv2 as cv

from FloorPlan import FloorPlan


from XdockParams import N_DOCK, TIME_ROLL_CONTAINER_LOAD, N_BUFFER_STORE
from SimulationConfig import set_dock_names_colors

def main():
    fp = FloorPlan(2*N_DOCK)
    set_dock_names_colors(fp)


    fp.draw(draw_grid=True)##draw_circulation=True, draw_grid=True)

    fp.imshow("Test")
    cv.setWindowTitle("Test", "X-dock")
    cv.moveWindow("Test", 10, 10)
    cv.waitKey(0)

    cv.destroyAllWindows()


if __name__ == "__main__":
    main()