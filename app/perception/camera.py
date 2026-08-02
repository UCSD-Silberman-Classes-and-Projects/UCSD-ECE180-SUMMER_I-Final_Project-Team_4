"""Camera capture on the MPU: USB webcam (V4L2) or a network MJPEG stream.

Isolates all capture details so the detector never touches the camera API.
Install OpenCV via apt (python3-opencv), not pip, on ARM/Debian.

Two source kinds, chosen by config.CAMERA_SOURCE (or the `source` argument):

  int   -- a V4L2 device index, e.g. 0 for /dev/video0 (Logitech USB webcam).
           Find yours with:  v4l2-ctl --list-devices
  str   -- a stream URL, e.g. "http://192.168.4.1:81/stream" for an ESP32-CAM
           running the standard CameraWebServer sketch.

Network streams are handled differently from USB in two ways that matter:
resolution is fixed by the sender (setting frame width/height locally does
nothing), and decoded frames queue up, so we keep the buffer at one frame and
always take the newest -- otherwise the robot acts on video that is seconds
old, which for an evasion loop is worse than no video at all.
"""

import cv2

from .. import config


def parse_source(text):
    """Turn a command-line value into a capture source.

    "0" -> 0 (device index);  "http://..." -> unchanged (stream URL).
    """
    if text is None:
        return None
    text = str(text)
    return int(text) if text.isdigit() else text


class Camera:
    def __init__(self, source=None, width=config.FRAME_WIDTH,
                 height=config.FRAME_HEIGHT):
        if source is None:
            source = getattr(config, "CAMERA_SOURCE", config.CAMERA_INDEX)
        self.source = source
        self.is_stream = isinstance(source, str)

        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            hint = ("Is the URL reachable? Try it in a browser first."
                    if self.is_stream else
                    "Check 'v4l2-ctl --list-devices' for the right index.")
            raise RuntimeError(f"Could not open camera source {source!r}. {hint}")

        # Always process the freshest frame; stale frames are useless here.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.is_stream:
            # Only meaningful for a local capture device.
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def read(self):
        """Return the latest frame (BGR ndarray), or None on failure."""
        ok, frame = self.cap.read()
        return frame if ok else None

    def actual_size(self):
        """(width, height) the source is really delivering.

        A network stream ignores the requested size, and some webcams silently
        fall back to a size they support, so read it back rather than assuming.
        """
        return (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    def release(self):
        self.cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.release()
