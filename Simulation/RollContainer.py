import cv2 as cv
from FloorPlan import W_ROLL_CONTAINER, H_ROLL_CONTAINER, BLACK



class RollContainer:
    def __init__(self, w, h, orientation, destination, shift, color):
        self.w         = w
        self.h         = h
        self.o         = orientation
        self.dest      = destination
        self.col       = color
        self.shift     = shift
        self.scheduled = False

    def draw(self, floor_plan):
        w1 = self.w - 0.5*W_ROLL_CONTAINER
        w2 = w1 + W_ROLL_CONTAINER
        h1 = self.h - 0.5*H_ROLL_CONTAINER
        h2 = h1 + H_ROLL_CONTAINER

        pt1 = floor_plan.pnt_from_coords(w1, h1)
        pt2 = floor_plan.pnt_from_coords(w2, h2)

        floor_plan.figure = cv.rectangle(floor_plan.figure, pt1, pt2, self.col, -1)
        f = 0.8
        if self.o==0: # left
            pt = floor_plan.pnt_from_coords(f*w2+(1-f)*w1, (h1+h2)/2)
        elif self.o==1: # up
            pt = floor_plan.pnt_from_coords((w1+w2)/2, f*h2+(1-f)*h1)
        elif self.o==2: # right
            pt = floor_plan.pnt_from_coords(f*w1+(1-f)*w2, (h1+h2)/2)
        elif self.o==3: # down
            pt = floor_plan.pnt_from_coords((w1+w2)/2, f*h1+(1-f)*h2)
        else:
            return

        floor_plan.figure = cv.circle(floor_plan.figure, pt, 5, (100, 100, 100), -1)

        text = f"{self.shift:d}"
        font = cv.FONT_HERSHEY_SIMPLEX
        floor_plan.figure = cv.putText(floor_plan.figure, text, pt1, font, 0.7, BLACK)

    def rotate(self, left):
        step   = 1 if left else 3
        self.o = (self.o + step)%4