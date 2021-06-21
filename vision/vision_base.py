#!/usr/bin/env python
# encoding: utf-8

import PIL.Image
import PIL.ImageStat
import PIL.ImageEnhance
import copy
import cv2
import math
import numpy as np
import time
import queue

low_blue = np.array([95, 80, 2])
high_blue = np.array([130, 255, 255])


class Vision:
    cam_width = 320
    cam_height = 160
    # mode = ai.status.Status.NORMAL.value
    cam_open = False
    head_moving = False # is the head moving? assist moving object detection

    def __init__(self):
        print("Vision start init")
        # face recognition
        self.recognizer = cv2.face.LBPHFaceRecognizer_create()
        self.recognizer.read('vision/trainer/trainer.yml')
        cascade_path = "vision/trainer/haarcascade_frontalface_default.xml"
        self.face_detector = cv2.CascadeClassifier(cascade_path)
        # object recognition
        self.previous_frame = []   # previous frame
        self.MIN_COUNTOUR_COUNT = 2   # min number of contours
        self.MAX_COUNTOUR_COUNT = 4   # max number of contours
        self.MIN_CONTOUR_AREA = 300   # min contour area
        self.MAX_CONTOUR_AREA = 1800   # max single contour area
        self.MAX_CONTOUR_DISTANCE = 60   # max distance between two centers of contours
        self.MIN_DETECT_RATIO = 0.57   # ratio of successful detections
        self.MIN_DETECT_ATTEMPTS = 3   # how many times to detect
        self.MOUSE_AREA_RANGE = [1500,2700]
        self.FISH_AREA_RANGE = [3000, 5200]
        self.MAX_VERTICAL_DIFFERENCE = 100
        self.MAX_HORIZONTAL_DIFFERENCE = 100
        self.MAX_SIZE_DIFFERENCE = 2500
        self.last_x = -1
        self.last_y = -1
        self.dis = 0
        self.counter = 0
        self.min_occupy = 0.7
        self.len_count = 15
        self.object_count = 0
        self.cam_no = 0
        self.cam = None
        self.circle_Queue = queue.Queue(maxsize=1)
        self.last_circle = (0, (0, 0), time.time())
        self.now_circle = (0, (0, 0), time.time())
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.markerLength = 0.08   # -- Here, the measurement unit is metre.
        self.other_init()

    def other_init(self):
        img = self.get_frame()
        size = img.shape

        # Camera internals
        focal_length = size[1]
        center = (size[1] / 2, size[0] / 2)
        self.camera_matrix = np.array(
            [[focal_length, 0, center[0]], [0, focal_length, center[1]],
             [0, 0, 1]],
            dtype="double")

        calibrationFile = "calibrationFileName.xml"
        calibrationParams = cv2.FileStorage(calibrationFile,
                                            cv2.FILE_STORAGE_READ)
        self.dist_coeffs = calibrationParams.getNode("distCoeffs").mat()



    # @ai.static_vars.static_vars(last=0, img=None)
    def get_frame(self):
        if True or (time.time() - Vision.get_frame.last) > 0.2:
            cam = cv2.VideoCapture(self.cam_no)
            cam.set(3, Vision.cam_width)   # 1280
            cam.set(4, Vision.cam_height)   # 720
            Vision.get_frame.last = time.time()
            Vision.get_frame.img = cam.read()[1]
            cam.release()
        return Vision.get_frame.img

    # Brightness: will return 0~150
    def get_brightness(self):
        # get brightness
        img = self.get_frame()
        frame = PIL.Image.fromarray(img).convert('RGB')
        stat = PIL.ImageStat.Stat(frame)
        r, g, b = stat.mean
        brightness = math.sqrt(0.299 * (r ** 2) + 0.587 * (g ** 2) + 0.114 *
                               (b ** 2))
        # create feature
        data = {}
        data['brightness'] = brightness
        return data

    def get_moving_distance(self, input_c1, input_c2):
        x_1 = input_c1[0]
        y_1 = input_c1[1]
        x_2 = input_c2[0]
        y_2 = input_c2[1]
        return math.sqrt((x_1 - x_2) * (x_1 - x_2) + (y_1 - y_2) * (y_1 - y_2))

    def sorted_circle(self, circles):
        sorted_circles = sorted(circles, key=lambda x: x[0], reverse=True)
        for index, circle in enumerate(sorted_circles[1:]):
            if sorted_circles[index - 1][0] == sorted_circles[index][0]:
                pass

    @staticmethod
    def count_circle_area(mask, radius, point):
        """
        :param mask: the color inrange mask
        :param radius: the radius of circle
        :param point:  the center point of circle
        :return: a list each point in circle,[0 or 1] 0 means the point
                 is black otherwise color
        """
        center_x = point[0]
        center_y = point[1]
        circle_color_list = []
        for x in range(center_x - radius, center_x + radius, 5):
            for y in range(center_y - radius, center_y + radius, 5):
                in_circle = (x - center_x) ** 2 + (
                    y - center_y) ** 2 <= radius ** 2
                if x >= 640 or y >= 480:
                    return circle_color_list
                if in_circle and mask[y][x] == 255:
                    circle_color_list.append(1)
                else:
                    circle_color_list.append(0)

        return circle_color_list

    def draw_circle(self, img):
        if not self.circle_Queue.empty():
            self.now_circle = self.circle_Queue.get()
        else:
            if time.time() - self.last_circle[-1] > 1:
                self.now_circle = (0, (0, 0), time.time())

            # self.now_circle = (0, (0, 0), time.time())

        if time.time() - self.last_circle[-1] > 1 and self.now_circle[0] == 0:
            self.last_circle = (0, (0, 0), time.time())

        #print(self.last_circle, self.now_circle)
        if self.last_circle[0] == 0:
            self.last_circle = self.now_circle

        else:
            if abs(self.last_circle[0]-self.now_circle[0]) < 1\
                and (abs(self.last_circle[1][0]-self.now_circle[1][0]) < 20)\
                and (abs(self.last_circle[1][1]-self.now_circle[1][1]) < 20):
                self.last_circle = self.now_circle

        circle = self.last_circle
        radius, center = circle[0], circle[1]
        cv2.circle(img, center, radius, (0, 255, 0), 2)

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # qrcode
        corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.arucoParams)
        # face
        fliped_gray = cv2.flip(gray,0)
        faces = self.face_detector.detectMultiScale(
            fliped_gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(20,20)
        )
        # moving obj
        frame2 = cv2.GaussianBlur(gray, (35, 35), 0)   # 15 15
        # cv2.imshow("blur",frame2)
        if len(self.previous_frame) is 0:
            self.previous_frame = copy.deepcopy(frame2)
        frame3 = cv2.absdiff(self.previous_frame, frame2)
        # cv2.imshow("abs diff",frame3)
        frame4 = cv2.threshold(frame3, 13, 255, cv2.THRESH_BINARY)[1]
        # cv2.imshow("threshold",frame4)
        kernel = np.ones((2, 2), np.uint8)
        frame5 = cv2.erode(frame4, kernel, iterations=4)
        # cv2.imshow("erode",frame5)
        frame5 = cv2.dilate(frame5, kernel, iterations=5)
        # cv2.imshow("dilate",frame5)
        # find contours on thresholded image
        contours, hierarchy = cv2.findContours(frame5.copy(),
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
       
        # ball
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gaus = cv2.GaussianBlur(hsv, (3, 3), 0)
        th = cv2.inRange(gaus, low_blue, high_blue)        
        mask = cv2.erode(th, None, iterations=4)
        dilate = cv2.dilate(mask,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                      (3, 3)),
                            iterations=4)
        dilate = cv2.GaussianBlur(dilate, (7, 7), 0)        
        circles = cv2.HoughCircles(dilate,
                                   method=cv2.HOUGH_GRADIENT,
                                   dp=1.2,
                                   minDist=70,
                                   param1=20,
                                   param2=35,
                                   minRadius=10,
                                   maxRadius=150)   # param1=15, param2=7,
        return corners, ids, frame2, contours, circles, faces, gray

    def if_has_qrcode(self, data, corners, ids):
        # if qrcode
        if len(corners) > 0:
            if ids is not None:   # if aruco marker detected
                data = int(ids[0][0])
                return {"type":"qrcode","data":data}
        else:
            return None

    def if_has_moving_obj(self, ft, contours):
        num_contours = len(contours)
        # print("num_contours:",num_contours)

        total_area = 0
        # Restricts the number of contours
        if not (self.MIN_COUNTOUR_COUNT <= num_contours <= self.MAX_COUNTOUR_COUNT ):
            self.object_count = 0
            self.counter = 0
            return None

        # loop over the contours
        contours_info = []
        for c in contours:
            # if the contour is too small, ignore it
            # print("single contour area:",cv2.contourArea(c))
            if cv2.contourArea(c) > self.MIN_CONTOUR_AREA:
                # contour data
                M = cv2.moments(c)   # ;print( M )
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                ca = cv2.contourArea(c)

                contours_info.append((cx, cy, ca))
                total_area += ca
                #print("(cx, cy, ca)",(cx, cy, ca))
                
        # contour area constriant
        if self.MOUSE_AREA_RANGE[0] <= total_area <= self.MOUSE_AREA_RANGE[1] or \
           self.FISH_AREA_RANGE[0] <= total_area <= self.FISH_AREA_RANGE[1]:
            # Sorts contours based on the area
            if not len(contours_info) < self.MIN_COUNTOUR_COUNT:
                
                contours_info.sort(key=lambda x:x[2], reverse=True)
                vectial_distance = abs( contours_info[0][1] - contours_info[1][1])
                horizontal_distance = abs( contours_info[0][0] - contours_info[1][0])
                size_diff = abs( contours_info[0][2] - contours_info[1][2])
                # print("vertical distance between these two largest contours:", vectial_distance)
                # print("horizontal distance between these two largest contours:", horizontal_distance)
                # print("size diff between these two largest contours:", size_diff)
                # print("\n")
                
                # Chooses shapes which are similar and close to each other
                if vectial_distance > self.MAX_VERTICAL_DIFFERENCE or \
                   horizontal_distance > self.MAX_HORIZONTAL_DIFFERENCE or \
                   size_diff > self.MAX_SIZE_DIFFERENCE:

                    return None
            else:
                return None

            self.object_count += 1
        else:
            self.object_count = 0
            self.counter = 0

        # Verify
        self.counter += 1
        if self.counter > self.MIN_DETECT_ATTEMPTS:
            obj_occu = self.object_count / self.MIN_DETECT_ATTEMPTS

            self.object_count = 0
            self.counter = 0

            if obj_occu >= self.MIN_DETECT_RATIO:
                obj_coords = [float(-1), float(-1)]
                data = [0, obj_coords]
                return {"type":"obj","data":data}
        else:
            return None

    def if_has_ball(self, data, circles):
        # if ball
        if circles is not None:
            x, y, radius = circles[0][0]
            # draw_circles = []
            data = [float(i) for i in [x, y, radius]]
            return {"type":"ball","data":data}
        else:
            return None

    def if_has_face(self, data, faces, gray, frame):
        # if face
        face_data = []
        confidence_min = 70
        confidence_max = 100
        for (x, y, w, h) in faces:
            person_id, confidence = self.recognizer.predict(gray[y:y + h,
                                                                 x:x + w])
            confidence = min(max(200 - confidence, 0), 100)
            if confidence_min < confidence < confidence_max:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, str(confidence), (x + 5, y + h - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 1)
                data = [(float(x), float(y)), confidence]
                face_data.append(data)

        if len(face_data) != 0:
            # data['human'] = face_data
            return {"type":"human","data":face_data}
        else:
            return None

    def get_face_ball_obj_qrcode(self):
        if not Vision.cam_open:
            Vision.cam_open = True
            self.cam = cv2.VideoCapture(self.cam_no)
            self.cam.set(3, Vision.cam_width)   # 1280
            self.cam.set(4, Vision.cam_height)   # 720
            # Vision.get_frame.img = cv2.flip(self.cam.read()[1], 0)
            print('Vision ------------> open cam')
        elif Vision.cam_open:
            Vision.cam_open = False
            self.cam.release()
            print('Vision ------------> close cam\n')

        if Vision.cam_open:
            # setups
            data = {}

            # frame = self.get_frame()
            frame = self.cam.read()[1]

            corners, ids, frame2, contours, circles, faces, gray = self.process_frame(
                frame)

            # order: face - ball - obj - QRcode
            # face
            ret = self.if_has_face(data, faces, gray, frame)
            if ret:
                return ret

            # ball
            ret = self.if_has_ball(data, circles)
            if ret:
                return ret

            # obj
            self.previous_frame = frame2
            ret = self.if_has_moving_obj(data, contours)
            if ret:
                return ret

            # QRcode
            ret = self.if_has_qrcode(data, corners, ids)
            if ret:
                return ret

            return None
        else:
            return None

    def test(self):
        self.cam = cv2.VideoCapture(self.cam_no)
        self.cam.set(3, Vision.cam_width)   # 1280
        self.cam.set(4, Vision.cam_height)   # 720
        print('Vision ------------> open cam')
        while (True):
            try:
                test_time = int(input("please how long to test (s): "))
                break
            except Exception as e:
                print('error: please input a number!')
                continue
        while (True):
            try:
                test_num = int(
                    input("please number(1:face, 2:ball, 3:obj, 4:qrcode): "))
                if test_num in [1, 2, 3, 4]:
                    break
                else:
                    continue
            except Exception as e:
                print('error: please input a number!')
                continue
        print('Vision ------------> will start test')
        time.sleep(1)

        start_time = time.time()
        while time.time() - start_time < test_time:
            result = None
            frame = self.cam.read()[1]
            corners, ids, frame2, contours, circles, faces, gray = self.process_frame(
                frame)
            data = {}

            if test_num == 1:   # face
                #cv2.imshow('result', frame)
                result = self.if_has_face(data, faces, gray, frame)

            elif test_num == 2:   # ball
                self.draw_circle(frame)
                #cv2.imshow('result', frame)
                result = self.if_has_ball(data, circles)

            elif test_num == 3:   # obj

                # update previous_frame
                cv2.drawContours(frame, contours, -1, (0,255,0), 3)
                # print("contours",contours)
                #cv2.imshow('result', frame)
                self.previous_frame = frame2
                result = self.if_has_moving_obj(data, contours)

            elif test_num == 4:   # qrcode
                #cv2.imshow('result', frame)
                result = self.if_has_qrcode(data, corners, ids)

            if result:
                print(str(result['type'])+' '+str(result['data']))

            k = cv2.waitKey(1) & 0xff
            if k == 27:
                break


if __name__ == '__main__':
    vision = Vision()
    vision.test()
