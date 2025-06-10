from asyncio import sleep
from multiprocessing import Process
from vuer import Vuer, VuerSession
from vuer.schemas import DefaultScene, Hands, WebRTCVideoPlane

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

        # Member variables
        self.webrtc_server_uri = "http://localhost:8080/offer"
        self.ngrok_webrtc_server_uri = "https://<ngrok_session_id>.ngrok-free.app/offer"

        # Start the Vuer app in a separate process
        self.process = Process(target=self.run)
        self.process.start()

    def run(self):
        """Run the Vuer app"""
        self.app_vuer.run()

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

        # Create camera stream plane
        quad = WebRTCVideoPlane(
            src=self.ngrok_webrtc_server_uri,
            key="video-quad",
            height=1,
            aspect=16 / 9,
            fixed=True,
            position=[0, 1, -2],
        )

        # Initialize the session
        session.set @ DefaultScene(quad, frameloop="always")
        session.upsert @ Hands(
            fps=30, stream=True, key="hands", showLeft=True, showRight=True
        )

        # Session loop
        while len(self.app_vuer.ws) > 0:
            # 'jpeg' encoding should give about 30fps with a 16ms wait in-between.
            await sleep(0.016 * 2)

        self.logger.info("WebSocket closed, exiting session loop")


if __name__ == "__main__":
    vuer_app = VuerApp()
