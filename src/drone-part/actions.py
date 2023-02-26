import cv2
import numpy as np
from timeit import default_timer as timer
import time
import math
import transformations as tf
from pid import PID


# Edge to screen ratio (for filtering)
EDGE = 0.02
# Chessboard edge length in meters
CHB_SIDE = 0.026
# Marker edge length in meters
MARKER_SIDE = 0.18
# Time delay
DELAY = 1.5
# Error allowed
ERROR = 0.15

# limit for averaging
ALLOW_LIMIT = 12

class Actions():
    def __init__(self, S, dir_queue, cam_data, getCoords_event, navigate_event, END_event):
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.cam_data = cam_data

        # for calibration
        self.db = 0
        self.chbEdgeLength = CHB_SIDE

        self.start = True
        self.tstart = 0
        self.calib = False

        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
        self.objp = np.zeros((6 * 9, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * self.chbEdgeLength

        # arrays to store object points and image points from all the images
        self.objpoints = []  # 3d point in real world space
        self.imgpoints = []  # 2d points in image plane.

        # for loading camera matrices
        self.not_loaded = True

        # drone navigation
        self.speed = S
        self.amplify = 10
        self.dir_queue = dir_queue
        self.getCoords_event = getCoords_event
        self.navigate_event = navigate_event
        self.END_event = END_event
        self.resetCoords = False
        self.t_lost = 0.
        self.t_inPos = 0.
        self.last_marker_pos = 1.
        self.beenThere = []
        self.TargetID = 1
        self.findNew = False

        # intiialise arrays
        self.ids = []
        self.tvec_origin = []
        self.rvec_origin = []
        self.dRot = []
        self.angle_origin = np.zeros((1, 3))
        self.height_origin = 0

        # for filtering values
        self.allow_use = []
        self.tvec_max = []
        self.tvec_min = []

        # the first markers orientation
        self.orientation = np.zeros((2, 3))

        # for logging
        self.OpenedFile = False

        # for data collection
        self.getCoords_event = getCoords_event

        # self.MarkerTarget = TargetDefine()

        # controller
        self.yaw_pid = PID(0.1, 0.00001, 0.001)
        self.v_pid = PID(0.5, 0.00001, 0.0001)
        self.vz_pid = PID(0.8, 0.00001, 0.0001)
        self.TargetPos = np.array([[0.2, 0.2, 2., 0.]])

        # for aruco markers
        self.markerEdge = MARKER_SIDE  # ArUco marker edge length in meters
        # self.seenMarkers = Markers(MARKER_SIDE, self.getCoords_event)
        self.getFirst = True

    # Calibrate camera
    def calibrator(self, frame):
        if self.calib == False:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (9,6), None)

        if self.db < 20:
            # if found, add object points, image points (after refining them)
            if ret == True and self.start:
                self.start = False
                self.tstart = time.time()
                self.objpoints.append(self.objp)

                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                self.imgpoints.append(corners2)

                # draw and display the corners
                frame = cv2.drawChessboardCorners(frame, (9,6), corners2, ret)
                self.db = self.db + 1
            elif ret == True and time.time() - self.tstart > 0.5:
                self.tstart = time.time()
                self.objpoints.append(self.objp)

                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                self.imgpoints.append(corners2)

                # draw and display the corners
                frame = cv2.drawChessboardCorners(frame, (9,6), corners2, ret)
                self.db = self.db + 1
            else:
                if ret == True:
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                    frame = cv2.drawChessboardCorners(frame, (9,6), corners2, ret)
                else:
                    cv2.putText(frame, "Please show chessboard.", (0, 64), self.font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            if self.calib == False:  # save the camera matrices first
                self.calib = True
                ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints,
                                                                                       gray.shape[::-1], None, None)
                h, w = frame.shape[:2]
                self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
                np.savez("calibration_files/camcalib", ret=ret, mtx=self.mtx, dist=self.dist, rvecs=self.rvecs,
                         tvecs=self.tvecs)

            # undistort
            frame = cv2.undistort(frame, self.mtx, self.dist, None, self.newcameramtx)
            # crop the image
            x, y, w, h = self.roi
            frame = frame[y:y + h, x:x + w]
            cv2.putText(frame, "Camera calibrated.", (0, 64), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        return frame

        # Append marker to the list of stered markers
    def appendMarker(self, seen_id_list, tvec, rvec, angles, tof):
        for n_id in seen_id_list:
            n_index = seen_id_list.index(n_id)
            if n_id == 1 and len(self.ids) == 0:
                # add the first marker as origin
                self.ids.append(n_id)
                self.tvec_origin.append(np.array([[0, 0, 0]]))
                self.rvec_origin.append(rvec[n_index])
                self.dRot.append(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
                self.allow_use.append(ALLOW_LIMIT)
                self.tvec_max.append(0)
                self.tvec_min.append(10000)
                self.angle_origin = angles
                self.height_origin = abs(tof)
                rvec_Euler = tf.rotationVectorToEulerAngles(rvec[n_index]) * 180 / math.pi
                # determine orientation
                orig_type = "?"
                if abs(rvec_Euler[0][0]) <= 150:  # horizontal
                    self.orientation = np.array([[1, 1, 1], [0, 1, 2]])
                    orig_type = "Horizontal"
                elif abs(rvec_Euler[0][0]) > 150:  # vertical
                    self.orientation = np.array([[1, -1, 1], [0, 2, 1]])
                    orig_type = "Vertical"
                # print(self.orientation)
                print(orig_type + " origin set")
            elif n_id not in self.ids and len(self.ids) > 0 and len(seen_id_list) >= 2:
                # append new marker to lists with dummy values
                self.ids.append(n_id)
                self.tvec_origin.append(np.array([[0, 0, 0]]))
                self.rvec_origin.append(np.array([[0, 0, 0]]))
                self.dRot.append(np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]))
                self.allow_use.append(0)
                self.tvec_max.append(0)
                self.tvec_min.append(10000)
                for m_id in seen_id_list:
                    if m_id in self.ids and m_id != n_id and self.allow_use[self.ids.index(m_id)] == ALLOW_LIMIT:
                        # n is to be added, m is already in list
                        m_index = seen_id_list.index(m_id)
                        m_index_list = self.ids.index(m_id)
                        n_index_list = self.ids.index(n_id)
                        # calculate needed matrix transformations
                        t, R, r, a, ma, mi = tf.getTransformations(n_id, tvec[m_index], tvec[n_index],
                                                                   rvec[m_index], rvec[n_index],
                                                                   self.tvec_origin[m_index_list],
                                                                   self.tvec_origin[n_index_list],
                                                                   self.rvec_origin[m_index_list],
                                                                   self.rvec_origin[n_index_list],
                                                                   self.dRot[m_index_list], self.dRot[n_index_list],
                                                                   self.allow_use[n_index_list], ALLOW_LIMIT,
                                                                   self.tvec_max[n_index_list],
                                                                   self.tvec_min[n_index_list])
                        self.tvec_origin[n_index_list] = t
                        self.dRot[n_index_list] = R
                        self.rvec_origin[n_index_list] = r
                        self.allow_use[n_index_list] = a
                        self.tvec_max[n_index_list] = ma
                        self.tvec_min[n_index_list] = mi
                        break
            elif n_id in self.ids and self.allow_use[self.ids.index(n_id)] < ALLOW_LIMIT:
                # marker can be used only after ALLOW_LIMIT has been reached
                for m_id in seen_id_list:
                    if m_id in self.ids and m_id != n_id and self.allow_use[self.ids.index(m_id)] == ALLOW_LIMIT:
                        # n is to be added, m is already in list
                        m_index = seen_id_list.index(m_id)
                        m_index_list = self.ids.index(m_id)
                        n_index_list = self.ids.index(n_id)
                        # calculate needed matrix transformations
                        t, R, r, a, ma, mi = tf.getTransformations(n_id, tvec[m_index], tvec[n_index],
                                                                   rvec[m_index], rvec[n_index],
                                                                   self.tvec_origin[m_index_list],
                                                                   self.tvec_origin[n_index_list],
                                                                   self.rvec_origin[m_index_list],
                                                                   self.rvec_origin[n_index_list],
                                                                   self.dRot[m_index_list], self.dRot[n_index_list],
                                                                   self.allow_use[n_index_list], ALLOW_LIMIT,
                                                                   self.tvec_max[n_index_list],
                                                                   self.tvec_min[n_index_list])
                        self.tvec_origin[n_index_list] = t
                        self.dRot[n_index_list] = R
                        self.rvec_origin[n_index_list] = r
                        self.allow_use[n_index_list] = a
                        self.tvec_max[n_index_list] = ma
                        self.tvec_min[n_index_list] = mi
                        break

    # Calculate camera pose from seen markers
    def getCoords(self, seen_id_list, tvecs, rvecs, angles):
        # print(seen_id_list, tvecs, rvecs, angles)
        length = len(seen_id_list)
        len_diff = 0
        dtv = np.zeros((1, 3))
        drv = np.zeros((1, 3))

        # calculating translation
        for i in range(length):
            if seen_id_list[i] in self.ids:
                ind = self.ids.index(seen_id_list[i])
                if self.allow_use[ind] == ALLOW_LIMIT:
                    dtv = dtv + tf.calculatePos(tvecs[i], rvecs[i], self.tvec_origin[ind], self.dRot[ind])
                else:
                    len_diff = len_diff + 1

        length = length - len_diff
        if length > 0:
            dtv = dtv / length

        # calculating rotation
        drv = angles - self.angle_origin

        return rvecs[0], tvecs[0]

    # Detect ArUco markers
    def aruco(self, frame, CoordGet, CoordReset, angles_tof):

        drv, dtv = [0,0,0], [0,0,0]
        # Get the calibrated camera matrices
        if self.not_loaded:
            with np.load(self.cam_data) as X:
                self.mtx = X['mtx']
                self.dist = X['dist']
            self.not_loaded=False

        h, w = frame.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(w,h),1,(w,h))

        # Undistort
        frame = cv2.undistort(frame, self.mtx, self.dist, None, newcameramtx)

        # Crop image
        x,y,w,h = roi
        frame = frame[y:y+h, x:x+w]
        #origFrame=np.copy(frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_100)
        parameters = cv2.aruco.DetectorParameters_create()

        # detecting markers: get corners and IDs
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        directions = np.zeros(4)


        # list for all currently seen IDs
        id_list=[]

        if CoordReset or self.resetCoords:
            print("Coordinates reset")
            self.resetCoords = False
            self.seenMarkers.nullCoords()

        if self.getCoords_event.is_set():
            CoordGet = True

        if np.all(ids != None):
            ### IDs found
            # pose estimation with marker edge length
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.markerEdge, self.mtx, self.dist)

            if len(ids) > 0:
                # from drone state queue
                angles = np.array([[angles_tof[0], angles_tof[1], angles_tof[2]]])
                tof = angles_tof[3]
                # add new marker
                self.appendMarker(id_list, tvecs, rvecs, angles, tof)
                # calculate positions, if marker is already allowed
                rvec, tvec = self.getCoords(ids, tvecs, rvecs, angles)

                R_matrix, _ = cv2.Rodrigues(rvec[0])
                R_matrix_transpose = np.array(R_matrix).T
                cam_translation = np.matmul(-R_matrix_transpose, tvec[0])

                print(tvec[0], rvec[0])
                print(cam_translation)

                # only selected vectors from now
                self.last_marker_pos = tvec[0][0]
                rvec = tf.rotationVectorToEulerAngles(rvec) * 180 / math.pi
                # flip the yaw angle if marker is upside down
                if abs(rvec[0][2]) > 90:
                    rvec[0][1] = -rvec[0][1]
                # print(tvec)
                # print(rvec)

                directions = [0., 0., 0., 0.]
                A = self.amplify * self.speed  # amplifier

                # yaw speed control
                err_yaw = rvec[0][1] - self.TargetPos[0][3]
                directions[3] = self.speed / 2 * self.yaw_pid.control(err_yaw)

                # vx, vy, vz speed control
                err_x = self.TargetPos[0][0] - tvec[0][0]
                directions[0] = -A * self.v_pid.control(err_x)
                err_y = self.TargetPos[0][2] - tvec[0][2]
                directions[1] = -A * self.v_pid.control(err_y)
                err_z = self.TargetPos[0][1] - tvec[0][1]
                directions[2] = A * self.vz_pid.control(err_z)

            for i in range(0, ids.size):
            #     cv2.aruco.drawAxis(frame, self.mtx, self.dist, rvecs[i], tvecs[i], 0.1)  # Draw axis
            #
                id_list.append(ids[i][0])

            # draw square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners)
        else:
            ### No IDs found
            cv2.putText(frame, "No Ids", (0,64), self.font, 1, (0,0,255),2,cv2.LINE_AA)
            if timer()-self.t_lost > 2:
                if self.last_marker_pos >= 0:
                    self.dir_queue.put([0, 0, 0, self.speed*2])
                else:
                    self.dir_queue.put([0, 0, 0, -self.speed*2])

        return frame, directions


