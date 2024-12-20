import cv2, numpy as np


class Aruco:
    def __init__(self, yaml, marker_size=22, dictionary=cv2.aruco.DICT_4X4_50):
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(dictionary),
                                                cv2.aruco.DetectorParameters())
        self.mtx = np.array(yaml.get('camera_matrix'))
        self.dist = np.array(yaml.get('dist_coeff'))
        self.marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                       [marker_size / 2, marker_size / 2, 0],
                                       [marker_size / 2, -marker_size / 2, 0],
                                       [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

    def detectMarkers(self, frame):
        _, ids, _ = self.detector.detectMarkers(frame)
        ret = []
        for i in ids:
            ret.append(int(i[0]))
        return ret

    def getXYZ(self, frame):
        corners, _, _ = self.detector.detectMarkers(frame)
        markers = []
        for c in corners:
            _, r, t = cv2.solvePnP(self.marker_points, c, self.mtx, self.dist, None, None, False,
                                   cv2.SOLVEPNP_IPPE_SQUARE)
            markers.append({'x': int(t[2]), 'y': int(0 - t[0]), 'z': int(0 - t[1] - 30), 'yaw': int(r[0])})
        return markers
