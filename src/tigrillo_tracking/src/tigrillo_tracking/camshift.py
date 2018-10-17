import numpy as np
import cv2
import copy


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
        self.term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

    def get_roi_hist(self, cam_shift):

        roi = cam_shift[self.r:self.r+self.h, self.c:self.c+self.w]
        hsv_roi =  cv2.cv2tColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
        roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180],[0,180])
        cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)

        return roi_hist

    def process(self, img):

        # Get region of interest
        if self.img_num == 0:
            self.roi_hist = self.get_roi_hist(img)        

        hsv = cv2.cv2tColor(img, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv],[0], self.roi_hist, [0,180], 1)

        # apply meanshift to get the new location
        self.ret, self.track_window = cv2.CamShift(dst, self.track_window, self.term_crit)

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

        pts = cv2.boxPoints(self.ret)
        pts = np.int0(pts)
        img2 = cv2.polylines(img, [pts], True, comp, 2)

        return img2


class CustomTracker:


    def __init__(self, part="front"):

        self.part = part

        self.ret = None
        self.img_num = 0
        self.img_tracked = None

    def process(self, img):

        # Convert to HSV
        img_hsv = copy.copy(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
        img_hsv[0:200, :] = 0

        # mask of red
        if self.part == "back":
            mask_red = cv2.inRange(img_hsv, (150, 150, 150), (180, 255,255))
            mask_red_2 = cv2.inRange(img_hsv, (0, 150, 150), (20, 255,255))
            imask = (mask_red > 0) + (mask_red_2 > 0) # + (mask_blue > 0)

        # mask of blue
        if self.part == "front":
            mask_blue = cv2.inRange(img_hsv, (85, 150, 150), (110, 255, 255))
            imask = (mask_blue > 0)
        
        # slice the color and convert to gray
        self.new = np.zeros_like(img, np.uint8)
        self.new[imask] = img[imask]
        self.img_grey = cv2.cvtColor(self.new, cv2.COLOR_BGR2GRAY)
        thresh, img_bw = cv2.threshold(self.img_grey, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        # erosion
        kernel = np.ones((3, 3), np.uint8)
        img_open = cv2.morphologyEx(img_bw, cv2.MORPH_OPEN, kernel)

        # dilatation
        self.img_dil = 255 - cv2.dilate(img_open, kernel, iterations=1)

        # find contours
        self.img_tracked, contours, hierarchy = cv2.findContours(self.img_dil, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        best_ctn = None
        best_ctn_val = 0
        for i, cnt in enumerate(contours):
            if 70 < cv2.contourArea(cnt) < 5000:
                if cv2.contourArea(cnt) > best_ctn_val:
                    best_ctn = cnt

        if best_ctn is not None:
            self.ret = cv2.minAreaRect(best_ctn)
        else:
            self.ret = ((0, 0), (0, 0), 0)
        return self.ret


    def draw(self, img, col="red"):

        comp = 0
        if col == "blue":
            comp = (255, 0, 0)
        elif col == "green":
            comp = (0, 255, 0)
        elif col == "red":
            comp = (0, 0, 255)

        pts = cv2.boxPoints(self.ret)
        pts = np.int0(pts)
        img2 = cv2.polylines(img, [pts], True, comp, 2)

        return img2