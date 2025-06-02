# Custom video track from ZED
from aiortc import VideoStreamTrack
import asyncio
from av import VideoFrame
import logging
import pyzed.sl as sl


class VideoTrackZED(VideoStreamTrack):
    def __init__(self):
        super().__init__()

        # Initialize logging
        self.logger = logging.getLogger(self.__class__.__name__)
        logging.basicConfig(level=logging.INFO)

        self.zed = sl.Camera()
        init = sl.InitParameters()
        if self.zed.open(init) != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("Failed to open ZED camera")
        self.image = sl.Mat()

    async def recv(self):
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
