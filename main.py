from djitellopy import Tello
import cv2
import yaml
import numpy as np
import time
import traceback

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, None, None, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash

tello = Tello()
tello.connect()
print(f"Battery: {tello.get_battery()}%\nTemp: {tello.get_temperature()}°C") #Вывод заряда и температуры дрона
markers = int(input("Кол-во объектов: "))
scan = int(input("Скорость сканирования: "))
markerSizeInCM = 22 #Установка размера маркера
with open('calibration.yaml') as f: #Загрузка матрицы искривления объектива камеры дрона
    loadeddict = yaml.safe_load(f)
mtx = loadeddict.get('camera_matrix')
dist = loadeddict.get('dist_coeff')
mtx = np.array(mtx)
dist = np.array(dist)
tello.streamon() #Включение камеры
frame_read = tello.get_frame_read()
try:
    tello.send_rc_control(0, 0, 0, 0)
    tello.takeoff()
    image = frame_read.frame
    dimensions = image.shape
    print(dimensions)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    for i in range(markers):
        tello.send_rc_control(0, 0, 0, scan)
        while True:
            image = cv2.cvtColor(frame_read.frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("Image", image)
            cv2.waitKey(1)
            (corners, ids, _) = detector.detectMarkers(image)
            if len(corners) > 0 and i in ids and len(ids) == 1:
                tello.send_rc_control(0, 0, 0, 0)
                print("Found marker")
                cv2.imshow("Image", image)
                cv2.waitKey(1)
                time.sleep(2)
                
                image = cv2.cvtColor(frame_read.frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("Image", image)
                cv2.waitKey(1)
                (corners, _, _) = detector.detectMarkers(image)
                rvec, tvec, _ = my_estimatePoseSingleMarkers(corners, markerSizeInCM, mtx, dist)
                tello.go_xyz_speed(int(tvec[0][0][2]) - 100, int(0 - tvec[0][0][0]), int(0 - tvec[0][0][1]) - 30, 50)
                
                image = cv2.cvtColor(frame_read.frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("Image", image)
                cv2.waitKey(1)
                (corners, _, _) = detector.detectMarkers(image)
                rvec, tvec, _ = my_estimatePoseSingleMarkers(corners, markerSizeInCM, mtx, dist)
                tello.go_xyz_speed(int(tvec[0][0][2]) - 30, int(0 - tvec[0][0][0]), int(0 - tvec[0][0][1]) - 30, 50)
                rotate = int(rvec[0][0][0])
                break
    if rotate > 0:
        tello.rotate_clockwise(rotate + 180)
    elif rotate < 0:
        tello.rotate_counter_clockwise(0 - (180 + rotate))
except Exception:
    print(traceback.format_exc())
    tello.send_rc_control(0, 0, 0, 0)
    tello.land()
    tello.streamoff()
    tello.end()
tello.send_rc_control(0, 0, 0, 0)
tello.land()
tello.streamoff()
tello.end()
