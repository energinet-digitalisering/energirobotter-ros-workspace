from asyncio import sleep
import logging
from multiprocessing import Process, Queue
import os
import signal
from vuer import Vuer
from vuer.events import Set
from vuer.schemas import DefaultScene, ImageBackground

import cv2


class VuerApp:
    def __init__(self):
        # Initialize logging
        self.logger = logging.getLogger("VuerApp")
        logging.basicConfig(level=logging.INFO)

        # Initialize the Vuer app
        self.app = Vuer()
        self.app.spawn(start=False)(self.set_vuer_images)

        self.queue_image_left = Queue(maxsize=2)
        self.queue_image_right = Queue(maxsize=2)

        # Start the Vuer app in a separate process
        self.process = Process(target=self.run)
        self.process.start()

        # Handle termination signals
        signal.signal(signal.SIGINT, self.cleanup)
        signal.signal(signal.SIGTERM, self.cleanup)

    def __del__(self):
        # Ensure cleanup on object deletion
        self.cleanup()

    def run(self):
        # Run the Vuer app
        self.app.run()

    def update_frames(self, left, right):

        # Update the image queues with new frames
        if self.queue_image_left.full():
            self.queue_image_left.get()
        self.queue_image_left.put(left)

        if self.queue_image_right.full():
            self.queue_image_right.get()
        self.queue_image_right.put(right)

    async def set_vuer_images(self, session):
        """Handle session lifecycle and set Vuer images."""
        try:
            # Initialize the session
            session @ Set(
                DefaultScene(up=[0, 1, 0]),
            )

            self.logger.info("New session started")

            # Ensure the WebSocket is active
            if len(self.app.ws) == 0:
                self.logger.warning("WebSocket session missing, ending session")
                return

            # Process images
            await self.process_images(session)

        except Exception as e:
            self.logger.error(f"Error in set_vuer_images: {e}", exc_info=True)

        finally:
            self.logger.info("Ending session processing")

    async def process_images(self, session):
        """Process image frames and send them to Vuer."""
        self.logger.info("Processing images")

        while len(self.app.ws) > 0:

            if self.queue_image_left.empty() or self.queue_image_right.empty():
                self.logger.warning("Empty image found, skipping frame update")
                continue

            image_left = self.queue_image_left.get(block=True)
            image_right = self.queue_image_right.get(block=True)

            if image_left is None or image_right is None:
                self.logger.warning("Image is None, skipping frame update")
                continue

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


if __name__ == "__main__":
    vuer_app = VuerApp()
