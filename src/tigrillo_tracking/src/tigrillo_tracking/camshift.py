import numpy as np
import cv2 as cv


class CamShift:

    def __init__(self, r, h, c, w):

        self.r = r
        self.h = h
        self.w = w
        self.c = c
        self.track_window = (self.c, self.r, self.w, self.h)
        self.ret = None

        self.img_num = 0

        # Termination criteria (10 its or moving)
        self.term_crit = ( cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 1 )

    def get_roi_hist(self, cam_shift):

        roi = cam_shift[self.r:self.r+self.h, self.c:self.c+self.w]
        hsv_roi =  cv.cvtColor(roi, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
        roi_hist = cv.calcHist([hsv_roi], [0], mask, [180],[0,180])
        cv.normalize(roi_hist, roi_hist, 0, 255, cv.NORM_MINMAX)

        return roi_hist

    def process(self, img):

        # Get region of interest
        if self.img_num == 0:
            self.roi_hist = self.get_roi_hist(img)        

        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        dst = cv.calcBackProject([hsv],[0], self.roi_hist, [0,180], 1)

        # apply meanshift to get the new location
        self.ret, self.track_window = cv.CamShift(dst, self.track_window, self.term_crit)

        # Increment image counter
        self.img_num += 1

        return self.ret

    def draw(self, img, col="red"):

        comp = 0
        if col == "blue":
            comp = (255, 0, 0)
        elif col == "green":
            comp = (0, 255, 0)
        elif col == "red":
            comp = (0, 0, 255)

        pts = cv.boxPoints(self.ret)
        pts = np.int0(pts)
        img2 = cv.polylines(img, [pts], True, comp, 2)

        return img2


