"""Color-based block detector using HSV thresholding and contour detection."""

import cv2
import numpy as np

COLOR_RANGES = {
    "red":    [(np.array([0, 100, 100]), np.array([10, 255, 255])),
               (np.array([170, 100, 100]), np.array([180, 255, 255]))],
    "green":  [(np.array([40, 80, 60]), np.array([80, 255, 255]))],
    "blue":   [(np.array([100, 120, 60]), np.array([130, 255, 255]))],
    "yellow": [(np.array([20, 100, 100]), np.array([35, 255, 255]))],
    "black":  [(np.array([0, 0, 0]), np.array([180, 255, 50]))],
}

MIN_CONTOUR_AREA = 500


def detect_blocks(frame_bgr, target_colors=None):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    colors = target_colors or list(COLOR_RANGES.keys())
    blocks = []

    for color in colors:
        ranges = COLOR_RANGES.get(color, [])
        mask = None
        for lower, upper in ranges:
            part = cv2.inRange(hsv, lower, upper)
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        if mask is None:
            continue

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2
            blocks.append({"color": color, "cx": cx, "cy": cy, "width": w, "height": h})

    return blocks


def annotate_frame(frame_bgr, blocks):
    annotated = frame_bgr.copy()
    cmap = {"red": (0, 0, 255), "green": (0, 255, 0),
            "blue": (255, 0, 0), "yellow": (0, 255, 255),
            "black": (128, 128, 128)}
    for b in blocks:
        c = cmap.get(b["color"], (255, 255, 255))
        x, y = b["cx"] - b["width"] // 2, b["cy"] - b["height"] // 2
        w, h = b["width"], b["height"]
        cv2.rectangle(annotated, (x, y), (x + w, y + h), c, 2)
        cv2.circle(annotated, (b["cx"], b["cy"]), 4, c, -1)
        cv2.putText(annotated, f"{b['color']} ({b['cx']},{b['cy']})",
                    (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
    return annotated
