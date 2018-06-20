import cv2
import time


class Cropper():

    def __init__(self, tracker, name):

        self.ix = 0
        self.iy = 0
        self.fx = 0
        self.fy = 0

        self.name = name
        self.tracker = tracker

        self.is_finished = False
        self.drawing = False

    def __on_mouse(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.ix, self.iy = x, y

        elif event == cv2.EVENT_MOUSEMOVE:
            self.tracker.img_can_show = True
            if self.drawing == True:
                cv2.rectangle(self.tracker.cv_img, (self.ix, self.iy), (x, y),(0,255,0), -1)
                self.tracker.img_can_show = True

        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.is_finished = True
            self.fx = x
            self.fy = y
            cv2.rectangle(self.tracker.cv_img, (self.ix, self.iy), (x, y),(0,255,0), -1)
            self.tracker.img_can_show = True

    def stop(self):

        cv2.setMouseCallback(self.name , lambda *args : None)

    def start(self):

        cv2.setMouseCallback(self.name , self.__on_mouse, 0)


class Selecter():

    def __init__(self, tracker, name):

        self.ix = 0
        self.iy = 0

        self.name = name
        self.tracker = tracker

        self.is_finished = False

    def __on_mouse(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.ix, self.iy = x, y
            cv2.rectangle(self.tracker.cv_img, (self.ix-8, self.iy-8), (self.ix+8, self.iy+8), (0,0,255), -1)
            self.tracker.img_can_show = True
            self.is_finished = True

    def get_points(self):

        return self.ix, self.iy
            
    def stop(self):

        cv2.setMouseCallback(self.name , lambda *args : None)

    def start(self):

        cv2.setMouseCallback(self.name , self.__on_mouse, 0)