#!/usr/bin/env python3

import cv2 as cv
import numpy as np


def analyze_arena(input_image):

    # ==========================================
    # LOAD IMAGE
    # ==========================================

    image = cv.imread(input_image)

    if image is None:

        print("Error loading image.")
        return {}

    # ==========================================
    # INITIALIZE OUTPUT
    # ==========================================

    result = {

        "arena_size": None,
        "start": None,
        "goal": None,
        "special_cells": {}

    }

    # ==========================================
    # WRITE YOUR LOGIC BELOW
    # ==========================================

    img_edges = cv.Canny(image, 50, 150)

    y_line = img_edges[img_edges.shape[0] // 2 + 10]

    x = img_edges.shape[0]

    edges = img_edges[37:x-37, 37:x-37]

    image = image[37:x-37, 37:x-37]

    l = edges[edges.shape[0] // 2 + 10].tolist()

    transitions = 0

    for i in range(1, len(y_line)):

        if y_line[i] == 255 and y_line[i-1] == 0:

            transitions += 1

    result["arena_size"] = transitions // 2 - 1

    result["start"] = ""

    result["goal"] = ""

    # estimated inner width
    a = int(
        (l.count(0) - 8 - 2 * result["arena_size"])
        / result["arena_size"]
    )

    # estimated cell step
    cell_size = a + 8

    # actual arena width
    arena_width = image.shape[1]

    # true geometric step
    true_step = arena_width / result["arena_size"]

    # proportional error
    per_cell_error = cell_size - true_step

    start_offset = int(a // 2 + 4)

    for i in range(result["arena_size"]):

        for j in range(result["arena_size"]):

            # original grid positions
            x_pos = start_offset + i * cell_size
            y_pos = start_offset + j * cell_size

            # proportional correction
            x = int(x_pos - i * per_cell_error)
            y = int(y_pos - j * per_cell_error)

            # small region for solid colors
            small_region = image[y-3:y+3, x-3:x+3]

            # large region for S/G text
            large_region = image[y-12:y+12, x-12:x+12]

            avg = np.mean(small_region, axis=(0,1))

            b, g, r = avg.astype(int)

            col = i
            row = result["arena_size"] - j

            coord = f"{chr(65 + col)}{row}"

            # yellow detection
            yellow_pixels = np.sum(

                (large_region[:,:,2] > 120) &
                (large_region[:,:,1] > 120) &
                (large_region[:,:,0] < 140)

            )

            # cyan detection
            cyan_pixels = np.sum(

                (large_region[:,:,0] > 120) &
                (large_region[:,:,1] > 120) &
                (large_region[:,:,2] < 140)

            )

            # RED → DANGER
            if r > 220 and g < 40 and b < 40:

                result['special_cells'][coord] = "DANGER"

            # GREEN → SAFE
            elif g > 220 and r < 40 and b < 40:

                result['special_cells'][coord] = "SAFE"

            # BLUE → REFUEL
            elif b > 220 and g < 40 and r < 40:

                result['special_cells'][coord] = "REFUEL"

            # ORANGE → SLOW
            elif r > 220 and 100 < g < 190 and b < 40:

                result['special_cells'][coord] = "SLOW"

            # START
            elif yellow_pixels > 5:

                result['start'] = coord

            # GOAL
            elif cyan_pixels > 5:

                result['goal'] = coord

    # ==========================================
    # SORT SPECIAL CELLS
    # ==========================================

    sorted_cells = dict(

        sorted(

            result["special_cells"].items(),

            key=lambda item: (

                item[0][0],
                int(item[0][1:])

            )
        )
    )

    result["special_cells"] = sorted_cells

    # ==========================================
    # RETURN FINAL OUTPUT
    # ==========================================

    return result
