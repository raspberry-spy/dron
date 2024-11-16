import yaml, cv2
from ArucoPnP import Aruco

with open('calibration.yaml') as f:
    loadeddict = yaml.safe_load(f)

aruco = Aruco(loadeddict, dictionary=cv2.aruco.DICT_6X6_50)

frame = cv2.imread('3.jpg')

print(aruco.getXYZ(frame))
