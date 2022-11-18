import cv2 as cv

from FloorPlan import FloorPlan


from XdockParams import N_DOCK
from Position import Position
from SimulationConfig import set_dock_names_colors

def main():
    fp = FloorPlan(2*N_DOCK)
    set_dock_names_colors(fp)

    fp.draw(draw_grid=True)##draw_circulation=True, draw_grid=True)

    pos1 = Position(fp, 12, parking=0)
    pos2 = Position(fp, 1, buffer_lane=3)
    path = fp.get_shortest_path(pos1, pos2)
    fp.draw_path(path, (0,0,255))

    fp.imshow("Test")
    cv.setWindowTitle("Test", "X-dock")
    cv.moveWindow("Test", 10, 10)
    cv.waitKey(0)

    cv.destroyAllWindows()


if __name__ == "__main__":
    main()