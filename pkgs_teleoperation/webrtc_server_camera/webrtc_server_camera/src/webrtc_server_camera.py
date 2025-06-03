"""
Original source: https://github.com/OpenTeleVision/TeleVision
"""

from aiortc import RTCPeerConnection, RTCSessionDescription
from aiohttp import web
import aiohttp_cors
from aiortc.rtcrtpsender import RTCRtpSender
import asyncio
import json
import os
import logging

from webrtc_server_camera.src.video_track_zed import VideoTrackZED


class WebRTCServerCamera:
    """
    A simple WebRTC server that streams video from a ZED camera using aiortc and aiohttp.
    """

    def __init__(self, host="0.0.0.0", port=8080, ssl_context=None):
        """
        Initialize the WebRTC server.

        Args:
            host (str): Host address to bind the server to. Defaults to "0.0.0.0".
            port (int): Port number to run the server on. Defaults to 8080.
            ssl_context (ssl.SSLContext, optional): SSL context for HTTPS. Defaults to None.
        """

        self.logger = logging.getLogger(self.__class__.__name__)
        logging.basicConfig(level=logging.INFO)

        self.host = host
        self.port = port
        self.ssl_context = ssl_context

        self.root = os.path.dirname(__file__)

        self.pcs = set()
        self.camera_track = VideoTrackZED()

        self.app = web.Application()
        self.app.on_shutdown.append(self.on_shutdown)

        cors = aiohttp_cors.setup(
            self.app,
            defaults={
                "*": aiohttp_cors.ResourceOptions(
                    allow_credentials=True,
                    expose_headers="*",
                    allow_headers="*",
                    allow_methods="*",
                )
            },
        )
        cors.add(self.app.router.add_get("/", self.index))
        cors.add(self.app.router.add_get("/client.js", self.javascript))
        cors.add(self.app.router.add_post("/offer", self.offer))

        self.runner = web.AppRunner(self.app)

    async def start(self):

        await self.runner.setup()
        site = web.TCPSite(
            self.runner, host=self.host, port=self.port, ssl_context=self.ssl_context
        )
        await site.start()
        self.logger.info(f"WebRTC server started at http://{self.host}:{self.port}")

    async def stop(self):
        self.logger.info("Stopping WebRTC server...")
        await self.app.shutdown()
        await self.app.cleanup()  # Triggers `on_shutdown`
        await self.runner.cleanup()

    async def on_shutdown(self, app):
        """
        Cleanup function called on server shutdown. Closes all peer connections.

        Args:
            app (aiohttp.web.Application): The aiohttp application instance.
        """

        self.logger.info("Shutting down. Closing peer connections...")

        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()

    def force_codec(self, pc, sender, forced_codec):
        """
        Force a specific codec for a given RTCRtpSender.

        Args:
            pc (RTCPeerConnection): The peer connection to modify.
            sender (RTCRtpSender): The RTP sender whose codec to set.
            forced_codec (str): MIME type of the codec to enforce, e.g., "video/H264".
        """

        kind = forced_codec.split("/")[0]
        codecs = RTCRtpSender.getCapabilities(kind).codecs
        transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
        transceiver.setCodecPreferences(
            [codec for codec in codecs if codec.mimeType == forced_codec]
        )

    async def index(self, request):
        """
        Serve the main HTML page.

        Args:
            request (aiohttp.web.Request): The incoming HTTP GET request.

        Returns:
            aiohttp.web.Response: The HTML page content.
        """

        content = open(os.path.join(self.root, "index.html"), "r").read()
        return web.Response(content_type="text/html", text=content)

    async def javascript(self, request):
        """
        Serve the client-side JavaScript.

        Args:
            request (aiohttp.web.Request): The incoming HTTP GET request.

        Returns:
            aiohttp.web.Response: The JavaScript file content.
        """

        content = open(os.path.join(self.root, "client.js"), "r").read()
        return web.Response(content_type="application/javascript", text=content)

    async def offer(self, request):
        """
        Handle incoming WebRTC offer from a client, respond with an answer.

        Args:
            request (aiohttp.web.Request): The incoming HTTP POST request containing SDP offer.

        Returns:
            aiohttp.web.Response: JSON response containing the SDP answer.
        """

        self.logger.info("Client connected")

        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        # Create and store peer connection
        pc = RTCPeerConnection()
        self.pcs.add(pc)

        # Setup cleanup on connection state change
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            self.logger.info(f"Connection state is {pc.connectionState}")
            if pc.connectionState in ("failed", "disconnected", "closed"):
                await pc.close()
                self.pcs.discard(pc)

        # Add video track
        video_sender = pc.addTrack(self.camera_track)
        self.force_codec(pc, video_sender, "video/H264")

        # Perform SDP negotiation
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
            ),
        )


if __name__ == "__main__":

    server = WebRTCServerCamera()

    async def main():
        await server.start()
        try:
            # Keep running until Ctrl+C
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            await server.stop()

    asyncio.run(main())
