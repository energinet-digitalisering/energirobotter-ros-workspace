from asyncio import sleep
import logging
from multiprocessing import Array, Process, Queue
import numpy as np
import os
import signal
from vuer import Vuer, VuerSession
from vuer.events import Set
from vuer.schemas import DefaultScene, Hands, ImageBackground


NR_OF_HAND_JOINTS = 25
HAND_JOINT_DATA_SIZE = 16


class VuerApp:
    def __init__(self):
        # Initialize logging
        self.logger = logging.getLogger("VuerApp")
        logging.basicConfig(level=logging.INFO)

        # Initialize the Vuer app
        self.app = Vuer()
        self.app.add_handler("HAND_MOVE")(self.on_hand_move)
        self.app.spawn(start=False)(self.session_manager)

        # Member variables
        self.queue_image_left = Queue(maxsize=2)
        self.queue_image_right = Queue(maxsize=2)

        self.hand_left_shared = Array(
            "d", (NR_OF_HAND_JOINTS * HAND_JOINT_DATA_SIZE), lock=True
        )
        self.hand_right_shared = Array(
            "d", (NR_OF_HAND_JOINTS * HAND_JOINT_DATA_SIZE), lock=True
        )

        # Start the Vuer app in a separate process
        self.process = Process(target=self.run)
        self.process.start()

        # Handle termination signals
        signal.signal(signal.SIGINT, self.cleanup)
        signal.signal(signal.SIGTERM, self.cleanup)

    def __del__(self):
        """Ensure cleanup on object deletion"""
        self.cleanup()

    def run(self):
        """Run the Vuer app"""
        self.app.run()

    def cleanup(self, signum=None, frame=None):
        """Clean up resources and terminate process."""
        self.logger.info("Cleaning up VuerApp...")

        # Terminate the process if it's still running
        if hasattr(self, "process") and self.process.is_alive():
            self.logger.info("Terminating subprocess...")
            self.process.terminate()
            self.process.join()

        # Close the image queues
        if not self.queue_image_left.empty():
            self.queue_image_left.close()

        if not self.queue_image_right.empty():
            self.queue_image_right.close()

        self.logger.info("Cleanup complete. Exiting.")

        # Explicitly exit when cleaning up from a signal
        if signum is not None:
            os._exit(0)

    def update_frames(self, left, right):
        """Update the image queues with new frames."""
        if self.queue_image_left.full():
            self.queue_image_left.get()
        self.queue_image_left.put(left)

        if self.queue_image_right.full():
            self.queue_image_right.get()
        self.queue_image_right.put(right)

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
        self.logger.info("Processing images")

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

            # Handle image queue
            if self.queue_image_left.empty() or self.queue_image_right.empty():
                self.logger.debug("Empty image found, skipping frame update")
                continue

            image_left = self.queue_image_left.get(block=True)
            image_right = self.queue_image_right.get(block=True)

            if image_left is None or image_right is None:
                self.logger.debug("Image is None, skipping frame update")
                continue

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

            # 'jpeg' encoding should give you about 30fps with a 16ms wait in-between.
            await sleep(0.016 * 2)

        self.logger.info("WebSocket closed, exiting image processing loop")

    @property
    def hand_left(self):
        return (
            np.array(self.hand_left_shared)
            .reshape((NR_OF_HAND_JOINTS, 4, 4))
            .transpose(0, 2, 1)
        )

    @property
    def hand_right(self):
        return (
            np.array(self.hand_right_shared)
            .reshape((NR_OF_HAND_JOINTS, 4, 4))
            .transpose(0, 2, 1)
        )


if __name__ == "__main__":
    vuer_app = VuerApp()
