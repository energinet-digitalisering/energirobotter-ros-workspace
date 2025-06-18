"""
VR Interface with Vuer.
It receives tracking data from the VR headset, and forwards a WebRTC camera stream to the headset, to update the texture of a fixed plane.
"""

import aiohttp
from aiohttp.web_response import Response
import asyncio
from cgi import parse_header
from multiprocessing import Process
import ngrok
import numpy as np
import traceback
from vuer import Vuer, VuerSession
from vuer.schemas import DefaultScene, Hands, WebRTCVideoPlane, WebRTCStereoVideoPlane

from teleoperation.src.vr_interface_app import VRInterfaceApp


class VuerApp(VRInterfaceApp):
    def __init__(self, camera_enabled=False, stereo_enabled=False, ngrok_enabled=False):
        VRInterfaceApp.__init__(self)

        self.camera_enabled = camera_enabled
        self.stereo_enabled = stereo_enabled
        self.ngrok_enabled = ngrok_enabled

        vuer_host = "localhost"
        vuer_port = 8012

        # URIs
        if camera_enabled:
            self.offer_route = "/offer"
            self.webrtc_server_uri = "http://localhost:8080" + self.offer_route

        # Establish ngrok connectivity
        if ngrok_enabled:
            self.ngrok_listener = ngrok.forward(vuer_port, authtoken_from_env=True)
            self.logger.info("----------------------------------------")
            self.logger.info(f"Connect to URL in headset: {self.ngrok_listener.url()}")
            self.logger.info("----------------------------------------")
        else:
            self.logger.info("----------------------------------------")
            self.logger.info(
                f"Connect to URL in headset: http://localhost:{vuer_port} (wired setup, remember to run `adb reverse tcp:{vuer_port} tcp:{vuer_port}` on connected computer)"
            )
            self.logger.info("----------------------------------------")

        # Initialize the Vuer app
        self.app_vuer = Vuer(
            host=vuer_host, port=vuer_port, free_port=True, static_root="."
        )
        self.app_vuer.add_handler("CAMERA_MOVE")(self.on_camera_move)
        self.app_vuer.add_handler("HAND_MOVE")(self.on_hand_move)
        self.app_vuer.spawn(start=False)(self.session_manager)

        # Add WebRTC offer proxy route
        if camera_enabled:
            self.app_vuer._route("/offer", self.proxy_offer, method="POST")

        # Start the Vuer app in a separate process
        self.process = Process(target=self.run)
        self.process.start()

    def run(self):
        """Run the Vuer app"""
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

        self.app_vuer.run()

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
        if self.camera_enabled:

            # Choose source
            if self.ngrok_enabled:
                stream_src = self.ngrok_listener.url() + self.offer_route
            else:
                stream_src = self.webrtc_server_uri

            # Create camera stream plane
            VideoPlaneClass = (
                WebRTCStereoVideoPlane if self.stereo_enabled else WebRTCVideoPlane
            )

            video_plane = VideoPlaneClass(
                src=stream_src,
                key="video-quad",
                height=1.5,
                aspect=16 / 9,
                fixed=True,
                position=[0, 1.0, -1.5],
                rotation=[np.deg2rad(-20), 0, 0],
            )

            # Set session again with video_plane
            session.set @ DefaultScene(video_plane, frameloop="always")

        # Add hand tracking
        session.upsert @ Hands(
            fps=30, stream=True, key="hands", showLeft=True, showRight=True
        )

        # Session loop
        while len(self.app_vuer.ws) > 0:
            # 'jpeg' encoding should give about 30fps with a 16ms wait in-between.
            await asyncio.sleep(0.016 * 2)

        self.logger.info("WebSocket closed, exiting session loop")


if __name__ == "__main__":
    vuer_app = VuerApp()
