"""Head-camera left-eye MJPEG streaming for the mapping-mode collection view.

Subscribes (ZMQ SUB) to the robot's head-camera JPEG stream (same source the
teleimager/YOLO stack uses), crops the left half of the side-by-side binocular
frame, and re-encodes it as an MJPEG multipart stream a browser <img> can show.

cv2/zmq are imported lazily so a camera problem never blocks backend startup.
"""
from .head_camera import HeadCameraStreamer

__all__ = ["HeadCameraStreamer"]
