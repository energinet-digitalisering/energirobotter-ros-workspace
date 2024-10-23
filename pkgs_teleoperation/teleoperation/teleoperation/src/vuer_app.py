from asyncio import sleep
from multiprocessing import Process, Queue

from vuer import Vuer
from vuer.schemas import Scene, ImageBackground


class VuerApp:
    def __init__(self, logger):

        self.logger = logger

        self.app = Vuer()
        self.app.spawn(start=False)(self.set_vuer_images)

        self.queue_image_left = Queue(maxsize=2)
        self.queue_image_right = Queue(maxsize=2)

        self.process = Process(target=self.run)
        self.process.start()

    def __del__(self):
        self.process.join()

    def run(self):
        self.app.run()

    def update_frames(self, left, right):
        if self.queue_image_left.full():
            self.queue_image_left.get()  # Remove the old frame
        self.queue_image_left.put(left)

        if self.queue_image_right.full():
            self.queue_image_right.get()  # Remove the old frame
        self.queue_image_right.put(right)

    async def set_vuer_images(self, session):
        session.set @ Scene()

        while True:

            if (not self.queue_image_left.empty()) and (
                not self.queue_image_right.empty()
            ):
                image_left = self.queue_image_left.get()
                image_right = self.queue_image_right.get()

                # use the upsert(..., to="bgChildren") syntax, so it is in global frame.
                session.upsert(
                    [
                        ImageBackground(
                            image_left,
                            aspect=1.778,
                            height=1,
                            distanceToCamera=1,
                            layers=1,
                            # One of ['b64png', 'png', 'b64jpeg', 'jpeg']
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
                            # One of ['b64png', 'png', 'b64jpeg', 'jpeg']
                            format="jpeg",
                            quality=90,
                            key="background-right",
                            interpolate=True,
                        ),
                    ],
                    to="bgChildren",
                )
                # 'jpeg' encoding should give you about 30fps with a 16ms wait in-between.
                # await sleep(0.0001)
                await sleep(0.016 * 2)


if __name__ == "__main__":
    vuer_app = VuerApp()
