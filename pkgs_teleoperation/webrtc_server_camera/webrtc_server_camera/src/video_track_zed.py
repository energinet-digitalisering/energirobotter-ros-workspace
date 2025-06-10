# Custom video track from ZED
from aiortc import VideoStreamTrack
import asyncio
from av import VideoFrame
import cv2
import numpy as np
import logging
import pyzed.sl as sl


class VideoTrackZED(VideoStreamTrack):
    """
    A custom WebRTC video stream track that captures frames from a ZED stereo camera.

    This class interfaces with the ZED SDK to grab frames from the left camera view
    and provides them as `aiortc`-compatible `VideoFrame` objects for streaming.
    """

    def __init__(self):
        """
        Initialize the ZED camera and prepare it for frame capture.

        Raises:
            RuntimeError: If the ZED camera fails to open.
        """

        super().__init__()

        self.logger = logging.getLogger(self.__class__.__name__)
        logging.basicConfig(level=logging.INFO)

        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        # init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = 30

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("Failed to open ZED camera")

        self.image_left = sl.Mat()
        self.image_right = sl.Mat()

    def __del__(self):
        """
        Closes ZED camera properly.
        """

        self.zed.close()

    async def recv(self):
        """
        Capture and return the next video frame from the ZED camera.

        This method is called by aiortc to retrieve the next frame to send to the client.
        It maintains a target framerate of 60 FPS.

        Returns:
            VideoFrame: The next frame to send, or None if the camera failed to grab a frame.
        """

        await asyncio.sleep(1 / 30)  # Maintain 30 FPS
        timestamp, time_base = await self.next_timestamp()

        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return None

        self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
        self.zed.retrieve_image(self.image_right, sl.VIEW.RIGHT)

        frame_left = self.image_left.get_data()
        frame_right = self.image_right.get_data()

        frame_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
        frame_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

        # Concatenate the images horizontally
        stereo_frame = np.hstack((frame_left, frame_right))

        stereo_frame = cv2.cvtColor(
            stereo_frame, cv2.COLOR_GRAY2BGR
        )  # To fit bgr24 format
        stereo_frame = cv2.resize(stereo_frame, (1280, 360))

        # Convert to VideoFrame
        video_frame = VideoFrame.from_ndarray(stereo_frame[:, :, :3], format="bgr24")
        video_frame.pts = timestamp
        video_frame.time_base = time_base
        return video_frame
