# utility functions for the 3pv drone project

import os
import yaml
from v4l2py.device import Device # For detecting camera port id
import numpy as np
import cv2


def read_camera_params(calibration_yaml:str):
    """
    Read camera params from a calibration .yaml file.

    Args:
        calibration_yaml (str): name of calibration file (e.g. "my_camera_calibration.yaml")

    Returns:
        tuple(np.array, np.array): camera_matirx, dist_coeffs
    """
    # construct path to find calibration yaml file
    path = os.path.join("/",*os.path.dirname(__file__).split("/")[1:-1], "config", calibration_yaml)

    # unpack calibration yaml file
    with open(path, "r") as stream:
        try:
            params = yaml.safe_load(stream)
            # read camera matrix and dist coeffs from parsed yaml data
            camera_matrix = np.array(params["camera_matrix"]["data"]).reshape(3, 3)
            dist_coeffs = np.array(params["distortion_coefficients"]["data"])
            return camera_matrix, dist_coeffs
        except yaml.YAMLError as exc:
            print(exc)

# function for finding id number of camera based on name
def get_camera_id(name_contains:str, port_range:int=10):
    """
    Find the first dev/ port number (id) of a usb camera whose card
    info contains the provided string.

    Args:
        name_contains (str, optional): identifying string in camera's v4l2 card info. Defaults to "USB".
        range (int): number of ports to check. Function will check ports 0, 1, 2... up to this number.

    Returns:
        int: id number (port number i guess)
    """
    cam_cards = {}
    match_id = None
    # loop through possible ids, starting at 0
    for id in range(port_range + 1):
        cam = Device.from_id(id)
        try:
            cam.open()
            cam_cards[id] = cam.info.card
            if name_contains in cam.info.card.lower() and not match_id:
                match_id = id
        except:
            pass
    
    print(f'searching for camera whose name contains "{name_contains}"')
    print("    available cameras:")
    for id, card in cam_cards.items():
        print(f"        id {id}:", card)
    
    if match_id is not None:
        print(f"search success:\n    id {match_id}: {cam_cards[match_id]}")
        return match_id
    else: # no match found
        raise ValueError(f'no camera found with name containing "{name_contains}"')

def estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return np.array(rvecs), np.array(tvecs), trash