"""
VR Interface with Vuer.
It receives tracking data from the VR headset, and forwards a WebRTC camera stream to the headset, to update the texture of a fixed plane.
"""

import aiohttp
from aiohttp.web_response import Response
from asyncio import sleep
from cgi import parse_header
from multiprocessing import Process
import traceback
from vuer import Vuer, VuerSession
from vuer.schemas import DefaultScene, Hands, WebRTCStereoVideoPlane

from teleoperation.src.vr_interface_app import VRInterfaceApp


class VuerApp(VRInterfaceApp):
    def __init__(self, camera_enabled=True):
        VRInterfaceApp.__init__(self)

        self.camera_enabled = camera_enabled

        # Initialize the Vuer app
        self.app_vuer = Vuer(host="0.0.0.0", port=8012, free_port=True, static_root=".")
        self.app_vuer.add_handler("CAMERA_MOVE")(self.on_camera_move)
        self.app_vuer.add_handler("HAND_MOVE")(self.on_hand_move)
        self.app_vuer.spawn(start=False)(self.session_manager)

        # Add WebRTC offer proxy route
        if camera_enabled:
            self.app_vuer._route("/offer", self.proxy_offer, method="POST")

        # Member variables
        self.webrtc_server_uri = "http://localhost:8080/offer"
        self.ngrok_webrtc_server_uri = "https://<ngrok_session_id>.ngrok-free.app/offer"

        # Start the Vuer app in a separate process
        self.process = Process(target=self.run)
        self.process.start()

    def run(self):
        """Run the Vuer app"""
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
        session.upsert @ Hands(
            fps=30, stream=True, key="hands", showLeft=True, showRight=True
        )

        if self.camera_enabled:
            # Create camera stream plane
            video_plane = WebRTCStereoVideoPlane(
                src=self.ngrok_webrtc_server_uri,
                key="video-quad",
                height=1,
                aspect=16 / 9,
                fixed=True,
                position=[0, 1, -1.5],
            )

            # Set session again with video_plane
            session.set @ DefaultScene(video_plane, frameloop="always")

        # Session loop
        while len(self.app_vuer.ws) > 0:
            # 'jpeg' encoding should give about 30fps with a 16ms wait in-between.
            await sleep(0.016 * 2)

        self.logger.info("WebSocket closed, exiting session loop")


if __name__ == "__main__":
    vuer_app = VuerApp()
