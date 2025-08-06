"""
VR Interface with Vuer.
It receives tracking data from the VR headset, and forwards a WebRTC camera stream to the headset, to update the texture of a fixed plane.
"""

import aiohttp
from aiohttp.web_response import Response
import asyncio
from cgi import parse_header
from enum import Enum
from multiprocessing import Process, Queue
import ngrok
import numpy as np
import traceback
from vuer import Vuer, VuerSession
from vuer.schemas import (
    DefaultScene,
    Hands,
    ImageBackground,
    WebRTCVideoPlane,
    WebRTCStereoVideoPlane,
)

from teleoperation.src.vr_interface_app import VRInterfaceApp


class CameraSource(Enum):
    NONE = None
    ROS = "ros"
    SERVER = "server"
    NGROK = "ngrok"

    @classmethod
    def from_input(cls, value, logger=None):
        if isinstance(value, cls):
            return value
        if isinstance(value, str):
            try:
                return cls(value)
            except ValueError:
                if logger:
                    logger.warning(
                        f"Camera source `{value}` not valid, starting without camera"
                    )
        return cls.NONE


class VuerApp(VRInterfaceApp):
    def __init__(self, camera_source=None, stereo_enabled=False):
        VRInterfaceApp.__init__(self)

        self.camera_source = CameraSource.from_input(camera_source, self.logger)
        self.stereo_enabled = stereo_enabled
        self.vuer_host = "0.0.0.0"
        self.vuer_port = 8012

        # Initialize the Vuer app
        self.app_vuer = Vuer(
            host=self.vuer_host,
            port=self.vuer_port,
            free_port=True,
            static_root=".",
            queries=dict(
                grid=False,
                collapseMenu=True,
            ),
        )

        self.app_vuer.add_handler("CAMERA_MOVE")(self.on_camera_move)
        self.app_vuer.add_handler("HAND_MOVE")(self.on_hand_move)
        self.app_vuer.spawn(start=False)(self.session_manager)

        # Camera setup, server configs, and URIs
        match self.camera_source:
            case CameraSource.ROS:
                self.queue_image_left = Queue(maxsize=2)
                self.queue_image_right = Queue(maxsize=2)
                self.log_localhost_instructions()

            case CameraSource.NGROK:
                # Establish ngrok connectivity
                self.ngrok_listener = ngrok.forward(
                    self.vuer_port,
                    domain="gladly-destined-lacewing.ngrok-free.app",
                    authtoken_from_env=True,
                )

                self.offer_route = "/offer"
                self.webrtc_server_uri_local = (
                    "http://localhost:8080" + self.offer_route
                )
                self.webrtc_server_uri = self.ngrok_listener.url() + self.offer_route

                # Add WebRTC offer proxy route
                self.app_vuer._route("/offer", self.proxy_offer, method="POST")

                self.logger.info("----------------------------------------")
                self.logger.info(
                    f"Connect to URL in headset: {self.ngrok_listener.url()}"
                )
                self.logger.info("----------------------------------------")

            case _:
                self.log_localhost_instructions()

        # Start the Vuer app in a separate process
        self.process = Process(target=self.run)
        self.process.start()

    def log_localhost_instructions(self):
        self.logger.info("----------------------------------------")
        self.logger.info(
            f"Connect to URL in headset: http://localhost:{self.vuer_port} "
            "(wired setup, remember to run `adb reverse tcp:{vuer_port} tcp:{vuer_port}` on connected computer)"
        )
        self.logger.info("----------------------------------------")

    async def proxy_offer(self, request):
        try:
            headers = {
                "Content-Type": request.headers.get("Content-Type", "application/json")
            }
            data = await request.read()

            async with aiohttp.ClientSession() as session:
                async with session.post(
                    self.webrtc_server_uri, data=data, headers=headers
                ) as resp:
                    response_data = await resp.read()

                    # Parse content type to extract 'type' and 'charset'
                    content_type_raw = resp.headers.get(
                        "Content-Type", "application/json"
                    )
                    content_type, params = parse_header(content_type_raw)

                    return Response(
                        body=response_data,
                        status=resp.status,
                        content_type=content_type,
                        charset=params.get("charset"),
                    )

        except Exception as e:
            traceback.print_exc()
            return aiohttp.web.Response(status=500, text=f"Proxy error: {e}")

    def run(self):
        """Run the Vuer app"""
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

        self.app_vuer.run()

    def update_frames(self, left, right):
        """Update the image queues with new frames."""

        if self.camera_source != CameraSource.ROS:
            return

        if self.queue_image_left.full():
            self.queue_image_left.get()
        self.queue_image_left.put(left)

        if self.queue_image_right.full():
            self.queue_image_right.get()
        self.queue_image_right.put(right)

    async def on_camera_move(self, event, session: VuerSession):
        """Handle head tracking data"""

        try:
            self.head_matrix_shared[:] = event.value["camera"]["matrix"]
        except Exception as e:
            self.logger.debug("Head not tracked: " + str(e))
            pass

    async def on_hand_move(self, event, session: VuerSession):
        """Handle hand tracking data"""
        # Left hand data
        try:
            self.hand_left_shared[:] = event.value["left"]
            self.logger.debug("Tracking left")
        except Exception as e:
            self.logger.debug("Left hand not tracked: " + str(e))
            pass

        # Right hand data
        try:
            self.hand_right_shared[:] = event.value["right"]
            self.logger.debug("Tracking right")
        except Exception as e:
            self.logger.debug("Right hand not tracked: " + str(e))
            pass

    async def session_manager(self, session: VuerSession):
        """Process image frames and send them to Vuer, as well as retrieving hand-tracking data."""
        self.logger.info("Session initialised")

        # Ensure the WebSocket is active
        if len(self.app_vuer.ws) == 0:
            self.logger.warning("WebSocket session missing, ending session")
            return

        # Initialize the session
        session.set @ DefaultScene(frameloop="always")

        # Setup camera stream plane
        if self.camera_source in [CameraSource.SERVER, CameraSource.NGROK]:

            # Create camera stream plane
            VideoPlaneClass = (
                WebRTCStereoVideoPlane if self.stereo_enabled else WebRTCVideoPlane
            )

            session.upsert @ VideoPlaneClass(
                src=self.webrtc_server_uri,
                key="video-quad",
                height=1.5,
                aspect=16 / 9,
                fixed=True,
                position=[0, 1.0, -1.5],
                rotation=[np.deg2rad(-20), 0, 0],
            )

        # Add hand tracking
        session.upsert @ Hands(
            fps=30, stream=True, key="hands", showLeft=True, showRight=True
        )

        # Session loop
        while len(self.app_vuer.ws) > 0:

            if self.camera_source in [CameraSource.ROS]:
                # Left camera
                if self.queue_image_left.empty():
                    self.logger.info("Left image empty, skipping frame update")
                    continue

                image_left = self.queue_image_left.get(block=True)

                if image_left is None:
                    self.logger.info("Left image is None, skipping frame update")
                    continue

                # Right camera
                if self.stereo_enabled:
                    if self.queue_image_right.empty():
                        self.logger.debug("Right image empty, skipping frame update")
                        continue

                    image_right = self.queue_image_right.get(block=True)

                    if image_right is None:
                        self.logger.debug("Right image is None, skipping frame update")
                        continue
                else:
                    image_right = image_left

                # Session content
                session.upsert(
                    [
                        ImageBackground(
                            image_left,
                            aspect=1.778,
                            height=1,
                            distanceToCamera=1,
                            layers=1,
                            format="jpeg",
                            quality=90,
                            key="background-left",
                            interpolate=True,
                        ),
                        ImageBackground(
                            image_right,
                            aspect=1.778,
                            height=1,
                            distanceToCamera=1,
                            layers=2,
                            format="jpeg",
                            quality=90,
                            key="background-right",
                            interpolate=True,
                        ),
                    ],
                    to="bgChildren",
                )

            # 'jpeg' encoding should give about 30fps with a 16ms wait in-between.
            await asyncio.sleep(0.016 * 2)

        self.logger.info("WebSocket closed, exiting session loop")


if __name__ == "__main__":
    vuer_app = VuerApp()
