'''This script is for generating data
1. Provide desired path to store images.
2. Press 'c' to capture image and display it.
3. Press any button to continue.
4. Press 'q' to quit.
'''

import cv2
from djitellopy import Tello

tello = Tello()
tello.connect()
print(f"Battery: {tello.get_battery()}%\nTemp: {tello.get_temperature()}°C")
tello.streamon()

#camera = cv2.VideoCapture(0)
frame_read = tello.get_frame_read()


path = r"C://Users//Admin//PycharmProjects//pythonProject1//camera_calibration-master//aruco_data//"
count = 29
while True:
    name = path + str(count)+".jpg"
    img = frame_read.frame
    cv2.imshow("img", img)


    if cv2.waitKey(20) & 0xFF == ord('c'):
        cv2.imwrite(name, img)
        cv2.imshow("img", img)
        count += 1
        print(count)
        if cv2.waitKey(0) & 0xFF == ord('d'):
            count = count - 1
