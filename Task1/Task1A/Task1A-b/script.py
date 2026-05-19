import cv2
import numpy as np


def analyze_video(video_path):

    # ==========================================
    # OUTPUT DICTIONARY
    # ==========================================

    result = {

        "top_wall_hits": 0,
        "bottom_wall_hits": 0,
        "left_wall_hits": 0,
        "right_wall_hits": 0

    }

    # ==========================================
    # OPEN VIDEO
    # ==========================================

    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():

        print("Error opening video")
        return result

    # ==========================================
    # GREEN COLOR RANGE (HSV)
    # ==========================================

    lower_green = np.array([35, 50, 50])
    upper_green = np.array([90, 255, 255])

    # ==========================================
    # FRAME DIMENSIONS
    # ==========================================

    WIDTH = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    HEIGHT = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # ==========================================
    # COLLISION FLAGS
    # ==========================================

    left_collision = False
    right_collision = False

    top_collision = False
    bottom_collision = False

    # ==========================================
    # WALL THRESHOLD
    # ==========================================

    wall_threshold = 4

    # ==========================================
    # PROCESS VIDEO
    # ==========================================

    while True:

        ret, frame = cap.read()

        if not ret:
            break

        # ==========================================
        # CONVERT FRAME TO HSV
        # ==========================================

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        # ==========================================
        # CREATE GREEN MASK
        # ==========================================

        mask = cv2.inRange(
            hsv,
            lower_green,
            upper_green
        )

        # ==========================================
        # FIND CONTOURS
        # ==========================================

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # ==========================================
        # BALL DETECTION AND COLLISION LOGIC
        # ==========================================

        if contours:

            # Find largest contour
            largest = max(
                contours,
                key=cv2.contourArea
            )

            # Ignore tiny contours/noise
            if cv2.contourArea(largest) > 50:

                # Get enclosing circle
                (center, radius) = cv2.minEnclosingCircle(
                    largest
                )

                cx = int(center[0])
                cy = int(center[1])

                radius = int(radius)

                # ==========================================
                # WALL TOUCH DETECTION
                # ==========================================

                touch_left = (
                    cx - radius <= wall_threshold
                )

                touch_right = (
                    cx + radius >= WIDTH - wall_threshold
                )

                touch_top = (
                    cy - radius <= wall_threshold
                )

                touch_bottom = (
                    cy + radius >= HEIGHT - wall_threshold
                )

                # ==========================================
                # COUNT COLLISIONS
                # ==========================================

                if touch_left and not left_collision:

                    result["left_wall_hits"] += 1

                if touch_right and not right_collision:

                    result["right_wall_hits"] += 1

                if touch_top and not top_collision:

                    result["top_wall_hits"] += 1

                if touch_bottom and not bottom_collision:

                    result["bottom_wall_hits"] += 1

                # ==========================================
                # UPDATE COLLISION FLAGS
                # ==========================================

                left_collision = touch_left

                right_collision = touch_right

                top_collision = touch_top

                bottom_collision = touch_bottom

    # ==========================================
    # RELEASE VIDEO
    # ==========================================

    cap.release()

    return result
