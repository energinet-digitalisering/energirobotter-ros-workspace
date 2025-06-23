from asyncio import sleep
from multiprocessing import Process, Queue
from vuer import Vuer, VuerSession
from vuer.schemas import DefaultScene, Hands, ImageBackground

from teleoperation.src.vr_interface_app import VRInterfaceApp


class VuerApp(VRInterfaceApp):
    def __init__(self, camera_enabled=True):
        VRInterfaceApp.__init__(self)

        self.camera_enabled = camera_enabled

        # Initialize the Vuer app
        self.app = Vuer(free_port=True)
        self.app.add_handler("CAMERA_MOVE")(self.on_camera_move)
        self.app.add_handler("HAND_MOVE")(self.on_hand_move)
        self.app.spawn(start=False)(self.session_manager)

        # Start the Vuer app in a separate process
        self.process = Process(target=self.run)
        self.process.start()

        # Member variables
        self.queue_image_left = Queue(maxsize=2)
        self.queue_image_right = Queue(maxsize=2)

    def run(self):
        """Run the Vuer app"""
        self.app.run()

    def update_frames(self, left, right):
        """Update the image queues with new frames."""
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

        # self.logger.info("States:")
        # self.logger.info(event.value["leftState"])
        # self.logger.info(event.value["rightState"])

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
        if len(self.app.ws) == 0:
            self.logger.warning("WebSocket session missing, ending session")
            return

        # Initialize the session
        session.set @ DefaultScene(frameloop="always")
        session.upsert @ Hands(
            fps=30, stream=True, key="hands", showLeft=True, showRight=True
        )

        # Session loop
        while len(self.app.ws) > 0:

            if self.camera_enabled:
                # Handle image queue
                if self.queue_image_left.empty():
                    self.logger.info("Left image empty, skipping frame update")
                    continue

                image_left = self.queue_image_left.get(block=True)

                if image_left is None:
                    self.logger.info("Left image is None, skipping frame update")
                    continue

                # if self.queue_image_right.empty():
                #     self.logger.debug("Right image empty, skipping frame update")
                #     continue

                # image_right = self.queue_image_right.get(block=True)

                # if image_right is None:
                #     self.logger.debug("Right image is None, skipping frame update")
                #     continue

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
                            image_left,
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
            await sleep(0.016 * 2)

        self.logger.info("WebSocket closed, exiting image processing loop")

if __name__ == "__main__":
    vuer_app = VuerApp()
