import cv2
import numpy as np

def map_arena():

    result = {
        "corner_points_detected": [],
        "robot_pixel_coord": [],
        "robot_real_world_coord": []
    }

    img = cv2.imread("test_images/angled_arena.png")

    if img is None:
        return result

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    red_lower1 = np.array([0, 100, 100])
    red_upper1 = np.array([10, 255, 255])

    red_lower2 = np.array([170, 100, 100])
    red_upper2 = np.array([180, 255, 255])

    green_lower = np.array([40, 50, 50])
    green_upper = np.array([90, 255, 255])

    blue_lower = np.array([100, 100, 50])
    blue_upper = np.array([140, 255, 255])

    yellow_lower = np.array([20, 100, 100])
    yellow_upper = np.array([35, 255, 255])

    red_mask = cv2.inRange(hsv, red_lower1, red_upper1) | \
               cv2.inRange(hsv, red_lower2, red_upper2)

    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

    def get_centroid(mask):
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:
            return [0, 0]

        largest = max(contours, key=cv2.contourArea)

        M = cv2.moments(largest)

        if M["m00"] == 0:
            return [0, 0]

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        return [cx, cy]

    red_pt = get_centroid(red_mask)
    green_pt = get_centroid(green_mask)
    blue_pt = get_centroid(blue_mask)
    yellow_pt = get_centroid(yellow_mask)

    result["corner_points_detected"] = [
        red_pt,      # Top-Left
        green_pt,    # Top-Right
        blue_pt,     # Bottom-Right
        yellow_pt    # Bottom-Left
    ]

    src_pts = np.float32([
        red_pt,
        green_pt,
        blue_pt,
        yellow_pt
    ])

    dst_pts = np.float32([
        [0, 0],
        [500, 0],
        [500, 500],
        [0, 500]
    ])

    H = cv2.getPerspectiveTransform(src_pts, dst_pts)

    warped = cv2.warpPerspective(
        img,
        H,
        (500, 500)
    )


    cv2.imwrite("birdsview.png", warped)

    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )

    detector_params = cv2.aruco.DetectorParameters()

    detector = cv2.aruco.ArucoDetector(
        aruco_dict,
        detector_params
    )

    corners, ids, _ = detector.detectMarkers(gray)

    robot_center = [0, 0]

    if ids is not None:

        for i, marker_id in enumerate(ids.flatten()):

            if marker_id == 1:

                pts = corners[i][0]

                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))

                robot_center = [cx, cy]
                break

    result["robot_pixel_coord"] = robot_center

    px, py = robot_center

    x_cm = round((px / 500.0) * 200.0, 1)
    y_cm = round((py / 500.0) * 200.0, 1)

    result["robot_real_world_coord"] = [x_cm, y_cm]

    return result


if __name__ == "__main__":
    output = map_arena()
    print("Task 2B Output:")
    print(output)
