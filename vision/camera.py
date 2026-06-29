"""USB Camera capture via OpenCV VideoCapture (Linux V4L2).

Uses Orbbec Gemini 335 as plain RGB camera. No pyorbbecsdk needed.
Device can be overridden via CAMERA_DEVICE env var (default /dev/video0).
"""

import os
import cv2
import numpy as np
import base64

CAMERA_DEVICE = os.environ.get("CAMERA_DEVICE", "/dev/video0")


class Camera:
    def __init__(self, device=CAMERA_DEVICE, width=640, height=480):
        self.device = device
        self.width = width
        self.height = height
        self._cap = None

    def _ensure_open(self):
        if self._cap is not None:
            return
        for dev in [self.device, 0, 1, 2]:
            cap = cv2.VideoCapture(dev)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            if cap.isOpened():
                self._cap = cap
                self.device = dev
                return
            cap.release()
        raise RuntimeError(
            f"Cannot open camera. Tried {self.device}, indices 0-2. "
            f"Set CAMERA_DEVICE env var or check USB connection."
        )

    def read(self):
        """Capture a single BGR frame. Returns (H, W, 3) numpy array."""
        self._ensure_open()
        ret, frame = self._cap.read()
        if not ret or frame is None:
            raise RuntimeError("Camera read returned no frame. Check USB connection.")
        return frame

    def read_jpeg_base64(self):
        """Capture and return as base64 JPEG string."""
        frame = self.read()
        ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            raise RuntimeError("Failed to encode frame as JPEG")
        return base64.b64encode(jpeg.tobytes()).decode("utf-8")

    def close(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __del__(self):
        self.close()
