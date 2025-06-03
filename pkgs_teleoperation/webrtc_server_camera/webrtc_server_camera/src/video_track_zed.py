# Custom video track from ZED
from aiortc import VideoStreamTrack
import asyncio
from av import VideoFrame
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
        init = sl.InitParameters()
        if self.zed.open(init) != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("Failed to open ZED camera")
        self.image = sl.Mat()

    async def recv(self):
        """
        Capture and return the next video frame from the ZED camera.

        This method is called by aiortc to retrieve the next frame to send to the client.
        It maintains a target framerate of 60 FPS.

        Returns:
            VideoFrame: The next frame to send, or None if the camera failed to grab a frame.
        """

        await asyncio.sleep(1 / 60)  # Maintain 60 FPS

        pts, time_base = await self.next_timestamp()

        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return None

        self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
        frame = self.image.get_data()

        # Convert to VideoFrame
        video_frame = VideoFrame.from_ndarray(frame[:, :, :3], format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame
