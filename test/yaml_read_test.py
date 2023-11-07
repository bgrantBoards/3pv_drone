import yaml
import os
import numpy as np

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

calibration_yaml = "dell_webcam_calibration.yaml"

(camera_matrix, dist_coeffs) = read_camera_params(calibration_yaml)

print("camera matrix:", camera_matrix, sep="\n")
print("distance coeffs:", dist_coeffs, sep="\n")