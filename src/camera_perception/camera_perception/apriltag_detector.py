import cv2
import numpy as np


def create_apriltag_detector():

    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11
    )

    parameters = cv2.aruco.DetectorParameters()

    parameters.cornerRefinementMethod = (
        cv2.aruco.CORNER_REFINE_SUBPIX
    )

    detector = cv2.aruco.ArucoDetector(
        dictionary,
        parameters
    )

    return detector


def detect_apriltags(frame, detector):

    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY
    )

    corners, ids, rejected = detector.detectMarkers(
        gray
    )

    detections = []

    if ids is None:
        return detections

    for marker_corners, marker_id in zip(
        corners,
        ids.flatten()
    ):

        points = marker_corners.reshape(
            4,
            2
        )

        center = np.mean(
            points,
            axis=0
        )

        center_u = int(center[0])
        center_v = int(center[1])

        detection = {
            'id': int(marker_id),
            'corners': points,
            'center': (
                center_u,
                center_v
            )
        }

        detections.append(
            detection
        )

    return detections