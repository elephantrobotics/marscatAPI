#!/usr/bin/python
# -*- coding: UTF-8 -*-
__author__ = "H.YL"

import math
import numpy as np
import cv2
import time
import threading
import sys
sys.path.append(".")
import ai.feature


class ArucoPoseEstimation(object):

    def __init__(self):
        """
        init the parameters
        """

        # --- to start real-time feed

        # --- calibration parameters
        calibrationFile = "calibrationFileName.xml"
        calibrationParams = cv2.FileStorage(calibrationFile,
                                            cv2.FILE_STORAGE_READ)
        self.dist_coeffs = calibrationParams.getNode("distCoeffs").mat()

        # --- importing aruco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.marker_length = 0.06   # -- Here, the measurement unit is metre.
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # --- 180 deg rotation matrix around the x axis
        # --- Use to get the attitude in terms of euler 321
        self.R_flip = np.zeros((3, 3), dtype=np.float32)
        self.R_flip[0, 0] = 1.0
        self.R_flip[1, 1] = -1.0
        self.R_flip[2, 2] = -1.0

        # use for save x,y,z,pitch
        self.pose_data = [None, None, None, None]
        self._id = [0]
        self.pose_data_dict = {}
        
        self.color_pile_data = [None, None]
        self._init()

    def _init(self):
        """TODO: Docstring for _init.

        :function: TODO
        :returns: TODO

        """
        cam = cv2.VideoCapture(0)
        # Capture frame-by-frame
        ret, frame = cam.read()
        self.width = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.count = cam.get(cv2.CAP_PROP_FRAME_COUNT)
        self.fps = cam.get(cv2.CAP_PROP_FPS)
        cam.release()
        size = frame.shape
        focal_length = size[1]
        center = (size[1] / 2, size[0] / 2)
        # Camera internals
        self.camera_matrix = np.array(
            [[focal_length, 0, center[0]], [0, focal_length, center[1]],
             [0, 0, 1]],
            dtype="double")

    def _is_rotation_matrix(self, R):
        """
        Checks if a matrix is a valid rotation matrix.

        :param R:    rotation matrix
        :return:     [bool] True or False
        """
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def _rotation_matrix_to_euler_angles(self, R):
        """
        Calculates rotation matrix to euler angles

        :param R:     rotation matrix
        :return:      [np.array] roll, pitch, yaw
        """
        assert (self._is_rotation_matrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def _flip_frame(self, frame, flipCode=-1):
        """
        Image mirrors

        :param frame:     Image frame
        :param flipCode:  Flip horizontal: flipCode = 1
                          Flip vertical: flipCode = 0
                          Flip both horizontally and vertically: flipCode = -1
        :return:          Image frame after flip
        """
        frame = cv2.flip(frame, flipCode)
        return frame

    def _detect(self, corners, ids, imgWithAruco):
        """
        Show the Axis of aruco and return the x,y,z(unit is cm), roll, pitch, yaw

        :param corners:        get from cv2.aruco.detectMarkers()
        :param ids:            get from cv2.aruco.detectMarkers()
        :param imgWithAruco:   assign imRemapped_color to imgWithAruco directly
        :return:               x,y,z (units is cm), roll, pitch, yaw (units is degree)
        """
        if len(corners) > 0:
            x1 = (corners[0][0][0][0], corners[0][0][0][1])
            x2 = (corners[0][0][1][0], corners[0][0][1][1])
            x3 = (corners[0][0][2][0], corners[0][0][2][1])
            x4 = (corners[0][0][3][0], corners[0][0][3][1])

            # Drawing detected frame white color
            # OpenCV stores color images in Blue, Green, Red
            cv2.line(imgWithAruco, x1, x2, (255, 0, 0), 1)
            cv2.line(imgWithAruco, x2, x3, (255, 0, 0), 1)
            cv2.line(imgWithAruco, x3, x4, (255, 0, 0), 1)
            cv2.line(imgWithAruco, x4, x1, (255, 0, 0), 1)

            # font type hershey_simpex
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(imgWithAruco, 'C1', x1, font, 1, (255, 255, 255), 1,
                        cv2.LINE_AA)
            cv2.putText(imgWithAruco, 'C2', x2, font, 1, (255, 255, 255), 1,
                        cv2.LINE_AA)
            cv2.putText(imgWithAruco, 'C3', x3, font, 1, (255, 255, 255), 1,
                        cv2.LINE_AA)
            cv2.putText(imgWithAruco, 'C4', x4, font, 1, (255, 255, 255), 1,
                        cv2.LINE_AA)

            if ids is not None:   # if aruco marker detected
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_length, self.camera_matrix,
                    self.dist_coeffs)

                # -- draw the coordinate of aruco
                imgWithAruco = cv2.aruco.drawAxis(imgWithAruco,
                                                  self.camera_matrix,
                                                  self.dist_coeffs, rvec, tvec,
                                                  self.marker_length)

                # --- The midpoint displays the ID number
                cornerMid = (int((x1[0] + x2[0] + x3[0] + x4[0]) / 4),
                             int((x1[1] + x2[1] + x3[1] + x4[1]) / 4))

                cv2.putText(imgWithAruco, "id=" + str(ids), cornerMid,
                            font, 1, (255, 255, 255), 1, cv2.LINE_AA)

                rvec = rvec[0][0]
                tvec = tvec[0][0]

                # --- Print the tag position in camera frame
                str_position = "MARKER Position x=%.4f (cm)  y=%.4f (cm)  z=%.4f (cm)" % (
                    tvec[0] * 100, tvec[1] * 100, tvec[2] * 100)

                # -- Obtain the rotation matrix tag->camera
                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T

                # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
                roll_marker, pitch_marker, yaw_marker = self._rotation_matrix_to_euler_angles(
                    self.R_flip * R_tc)

                # -- Print the marker's attitude respect to camera frame
                str_attitude = "MARKER Attitude degrees r=%.4f  p=%.4f  y=%.4f" % (
                    math.degrees(roll_marker), math.degrees(pitch_marker),
                    math.degrees(yaw_marker))
                '''
                print(str_position)
                print("rotation x=%.4f (degree) " % 
                      (math.degrees( math.atan(tvec[0]/tvec[2]))))
                print(str_attitude)
                print(math.degrees(pitch_marker)+math.degrees( math.atan(tvec[0]/tvec[2])))
                print("-----------------------------------------------")
                '''
                pose_data = [None, None, None, None]
                pose_data[0] = tvec[0] * 100
                pose_data[1] = tvec[1] * 100
                pose_data[2] = tvec[2] * 100
                pose_data[3] = math.degrees(pitch_marker)
                
                self.pose_data_dict[ids] = pose_data
                return (tvec[0] * 100, tvec[1] * 100, tvec[2] * 100), \
                       (math.degrees(roll_marker),
                        math.degrees(pitch_marker),
                        math.degrees(yaw_marker))
                

        else:
            self.pose_data[0] = None
            self.pose_data[1] = None
            self.pose_data[2] = None
            self.pose_data[3] = None
            #self.pose_data_list[0] = self.pose_data
            self.pose_data_dict[0] = self.pose_data
            return None
        

    def _color_detect_process(self, image):
        # RGB2HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Structural element
        line = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15), (-1, -1))
        # Define range of HSV
        mask = cv2.inRange(hsv, (0, 43, 46), (5, 255, 255))
        # Anticoincidence operation
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, line)

        # Contour extraction, find the maximum contour
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
        index = -1
        maxval = 0
        for c in range(len(contours)):
            area = cv2.contourArea(contours[c])
            if area > maxval:
                maxval = area
                index = c
        # Draw
        if index >= 0:
            rect = cv2.minAreaRect(contours[index])

            # Ellipse fitting
            cv2.ellipse(image, rect, (255, 0, 0), 2, 8)
            # center point positioning
            cv2.circle(image, (np.int32(rect[0][0]), np.int32(rect[0][1])), 2,
                       (0, 255, 0), 2, 8, 0)
            # print("(x, y)", rect[0][0] - int(self.width/2), rect[0][1]- int(self.height/2) )
            return image, [
                np.int32(rect[0][0] - int(self.width / 2)),
                np.int32(rect[0][1] - int(self.height / 2))
            ]

        return image, [None, None]

    def main(self):
        # Capture frame-by-frame

        cam = cv2.VideoCapture(0)
        ret, frame = cam.read()
        cam.release()
        # -- Frame mirros
        frame = self._flip_frame(frame, flipCode=-1)
        frame1, self.color_pile_data = self._color_detect_process(frame)

        # Our operations on the frame come here
        # aruco.detectMarkers() requires gray image
        imgRemapped_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # -- Detect aruco
        #    rejectImaPoint：
        #        contains the imgPoints of those squares whose
        #        inner code has not a correct codification.
        #        Useful for debugging purposes.
        corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
            imgRemapped_gray, self.aruco_dict, parameters=self.aruco_params)
        #print(f'**************rejectImaPoint: {rejectImaPoint}')
        if ids is not None:
            ids = ids.tolist()
            if len(ids)>0:
                for i in range(len(ids)):
                    self._id = [i[0] for i in ids]
        else:
            self._id = [0]

        # -- Draw the camera center point
        #self.pose_data_list = [None for i in range(len(self._id))]
        cv2.aruco.drawAxis(imgRemapped_gray, self.camera_matrix,
                           self.dist_coeffs, (0, 0, 0), (0, 0, 0), 0.1)

        # -- If the corners be detected，
        # Draw the Axis of aruco
        # and return the position (x, y, z)(cm) and rotation (roll, pitch, yaw)(degree)
        if ids is not None:
            if len(ids) > 0:
                for i in range(len(ids)):
                    a = ids[i][0]
                    self._detect(corners[i:i+1], a, imgRemapped_gray)
        else:
            self._detect(corners, ids, imgRemapped_gray)

            

        #cv2.imshow("aruco", imgRemapped_gray)   # display

    def get_pose_data(self):
        """
        Get the data of Pose in time

        :return: [None, None. None. None] or [x, y, z, delta]
        """
        # print(self.PoseData)
        ft = ai.feature.Feature()
        ft.type = ai.feature.Type.CHARGING.value
        ft.received_time = time.time()
        ft.timeout = 1.0
        ft.data['posedata'] = self.pose_data_dict
        ft.data['id'] = self._id
        self.pose_data_dict = {}
        return ft

    def get_color_pile_data(self):
        """
        Get the data of the charging pile of

        :return:  [None, None] or [x, y]
        """
        # print(self.PoseData)

        return self.color_pile_data


def test():
    import ai.action.move.chargingmove
    B = ai.action.move.chargingmove.BodyMove()
    B.headInit()
    B.staticPose()
    APE = ArucoPoseEstimation()
    th1 = threading.Thread(target=APE.main, daemon=True)
    th1.start()

    last = None
    while 1:
        a = APE.get_pose_data()
        if a.data['posedata'][0] is not None:
            if last != a.data['posedata'][0]:
                #print(a)
                last = a.data['posedata'][0]


if __name__ == "__main__":
    APE = ArucoPoseEstimation()
    APE.main()
