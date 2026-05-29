import cv2
import numpy as np
import glob
import os


def localize_bot():

    base_dir = os.path.dirname(os.path.abspath(__file__))
    #i added this as the path often times was not found

    checkerboard_size = (9, 6)
    square_size = 2.5  # cm

    objp = np.zeros(
        (checkerboard_size[0] * checkerboard_size[1], 3),
        np.float32
    )

    objp[:, :2] = np.mgrid[
        0:checkerboard_size[0],
        0:checkerboard_size[1]
    ].T.reshape(-1, 2)

    objp *= square_size
    objpoints = []
    imgpoints = []

    
    calibration_path = os.path.join(
        base_dir,
        "calibration_images",
        "*.png"
    )

    images = glob.glob(calibration_path)

    if not images:
        raise FileNotFoundError(
            f"No calibration images found in:\n{calibration_path}"
        )

    image_size = None

    for fname in images:

        img = cv2.imread(fname)

        if img is None:
            print(f"Warning: Could not read {fname}")
            continue

        gray = cv2.cvtColor(
            img,
            cv2.COLOR_BGR2GRAY
        )

        image_size = gray.shape[::-1]

        
        ret, corners = cv2.findChessboardCorners(
            gray,
            checkerboard_size,
            None
        )

        if ret:

            criteria = (
                cv2.TERM_CRITERIA_EPS +
                cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001
            )

            corners2 = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                criteria
            )

            objpoints.append(objp)
            imgpoints.append(corners2)

    
    if len(objpoints) == 0:
        raise ValueError(
            "No checkerboard corners were detected in any calibration image."
        )

    if image_size is None:
        raise ValueError(
            "Failed to determine calibration image size."
        )

   
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None
    )

    if not ret:
        raise RuntimeError(
            "Camera calibration failed."
        )

  
    camera_matrix_trace = round(
        float(np.trace(camera_matrix)),
        2
    )


    arena_path = os.path.join(
        base_dir,
        "test_images",
        "test_arena.jpg"
    )

    arena = cv2.imread(arena_path)

    if arena is None:
        raise FileNotFoundError(
            f"Could not load arena image:\n{arena_path}"
        )


    undistorted = cv2.undistort(
        arena,
        camera_matrix,
        dist_coeffs
    )

  
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )

    parameters = cv2.aruco.DetectorParameters()

    detector = cv2.aruco.ArucoDetector(
        aruco_dict,
        parameters
    )

    corners, ids, rejected = detector.detectMarkers(
        undistorted
    )

    result = {
        "camera_matrix_trace": camera_matrix_trace,
        "markers": {}
    }

 
    marker_size = 5.0

    objp_marker = np.array([
        [-marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0]
    ], dtype=np.float32)

  
    if ids is not None:

        for i, marker_id in enumerate(ids.flatten()):

            image_points = corners[i][0].astype(
                np.float32
            )

            success, rvec, tvec = cv2.solvePnP(
                objp_marker,
                image_points,
                camera_matrix,
                dist_coeffs
            )

            if not success:
                print(
                    f"Warning: solvePnP failed for marker {marker_id}"
                )
                continue

            distance_z = round(
                float(tvec[2][0]),
                1
            )

            x_offset = round(
                float(tvec[0][0]),
                1
            )

            result["markers"][f"id_{marker_id}"] = {
                "distance_z": distance_z,
                "x_offset": x_offset
            }

    return result


if __name__ == "__main__":

    output = localize_bot()
    print(output)
