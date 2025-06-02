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
    def __init__(self, host="0.0.0.0", port=8080, ssl_context=None):
        # Initialize logging
        self.logger = logging.getLogger(self.__class__.__name__)
        logging.basicConfig(level=logging.INFO)

        self.root = os.path.dirname(__file__)

        self.pcs = set()
        self.camera_track = VideoTrackZED()

        app = web.Application()
        app.on_shutdown.append(self.on_shutdown)

        cors = aiohttp_cors.setup(
            app,
            defaults={
                "*": aiohttp_cors.ResourceOptions(
                    allow_credentials=True,
                    expose_headers="*",
                    allow_headers="*",
                    allow_methods="*",
                )
            },
        )
        cors.add(app.router.add_get("/", self.index))
        cors.add(app.router.add_get("/client.js", self.javascript))
        cors.add(app.router.add_post("/offer", self.offer))

        web.run_app(app, host=host, port=port, ssl_context=ssl_context)

    def force_codec(self, pc, sender, forced_codec):
        kind = forced_codec.split("/")[0]
        codecs = RTCRtpSender.getCapabilities(kind).codecs
        transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
        transceiver.setCodecPreferences(
            [codec for codec in codecs if codec.mimeType == forced_codec]
        )

    async def offer(self, request):
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

    async def on_shutdown(self, app):
        self.logger.info("Shutting down. Closing peer connections...")

        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()

    async def index(self, request):
        content = open(os.path.join(self.root, "index.html"), "r").read()
        return web.Response(content_type="text/html", text=content)

    async def javascript(self, request):
        content = open(os.path.join(self.root, "client.js"), "r").read()
        return web.Response(content_type="application/javascript", text=content)


if __name__ == "__main__":
    webrtc_server_camera = WebRTCServerCamera()
