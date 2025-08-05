from aiohttp import ClientSession
from aiortc import (
    RTCIceCandidate,
    RTCPeerConnection,
    RTCSessionDescription,
    RTCConfiguration,
    RTCIceServer,
)
from aiortc.sdp import candidate_from_sdp
from aiortc.rtcrtpsender import RTCRtpSender
import asyncio
import json
import logging

from webrtc_client_camera.src.video_track_zed import VideoTrackZED


class WebRTCClientCamera:
    """
    A simple WebRTC client that streams video from a ZED camera.
    """

    def __init__(
        self,
        server_url="https://teleop.energirobotter.org/ws",
        stereo_enabled=False,
    ):
        self.logger = logging.getLogger(self.__class__.__name__)
        logging.basicConfig(level=logging.INFO)

        self.server_url = server_url
        self.stereo_enabled = stereo_enabled

        # Configure RTCPeerConnection with STUN server using RTCConfiguration
        self.pc = RTCPeerConnection(
            configuration=RTCConfiguration(
                iceServers=[RTCIceServer(urls=["stun:stun.l.google.com:19302"])]
            )
        )
        self.camera_track = VideoTrackZED(stereo_enabled)

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

    async def connect(self):
        # Add video track
        try:
            video_sender = self.pc.addTrack(self.camera_track)
            self.force_codec(self.pc, video_sender, "video/H264")
        except Exception as e:
            self.logger.error(f"Error adding video track: {e}")
            return

        async with ClientSession() as session:
            try:
                ws = await session.ws_connect(self.server_url)
                self.logger.info(f"Connected to WebSocket server: {self.server_url}")
            except Exception as e:
                self.logger.error(f"WebSocket connection failed: {e}")
                return

            # Optional: send pings to keep the WebSocket alive
            async def keep_alive():
                try:
                    while True:
                        await ws.ping()
                        await asyncio.sleep(20)  # Adjust as needed
                except asyncio.CancelledError:
                    self.logger.info("Ping loop cancelled.")
                except Exception as e:
                    self.logger.warning(f"Ping loop error: {e}")

            ping_task = asyncio.create_task(keep_alive())

            @self.pc.on("icecandidate")
            async def on_icecandidate(candidate):
                if candidate:
                    await ws.send_json({"candidate": candidate.to_dict()})
                    self.logger.info(f"Sent ICE candidate: {candidate.to_dict()}")

            @self.pc.on("connectionstatechange")
            async def on_connectionstatechange():
                self.logger.info(f"Connection state: {self.pc.connectionState}")
                if self.pc.connectionState == "failed":
                    self.logger.warning("Connection failed, closing.")
                    await self.pc.close()
                    await ws.close()

            try:
                async for msg in ws:
                    data = json.loads(msg.data)
                    if "sdp" in data and data["type"] == "offer":
                        offer_id = data.get("id")
                        sd = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                        try:
                            self.logger.info(f"Received offer SDP:\n{data['sdp']}")
                            await self.pc.setRemoteDescription(sd)
                            self.logger.info(
                                f"Signaling state after setRemoteDescription: {self.pc.signalingState}"
                            )
                            answer = await self.pc.createAnswer()
                            await self.pc.setLocalDescription(answer)
                            self.logger.info(f"Generated answer SDP:\n{answer.sdp}")
                            await ws.send_json(
                                {"type": "answer", "id": offer_id, "sdp": answer.sdp}
                            )
                            self.logger.info(f"Sent SDP answer with id: {offer_id}")
                        except Exception as e:
                            self.logger.error(f"Error processing offer: {e}")
                    elif "candidate" in data:
                        cand = data["candidate"]
                        if (
                            "candidate" in cand
                            and "sdpMid" in cand
                            and "sdpMLineIndex" in cand
                        ):
                            parsed = candidate_from_sdp(cand["candidate"])
                            candidate = RTCIceCandidate(
                                component=parsed.component,
                                foundation=parsed.foundation,
                                ip=parsed.ip,
                                port=parsed.port,
                                priority=parsed.priority,
                                protocol=parsed.protocol,
                                type=parsed.type,
                                sdpMid=cand["sdpMid"],
                                sdpMLineIndex=cand["sdpMLineIndex"],
                                tcpType=parsed.tcpType,
                                relatedAddress=parsed.relatedAddress,
                                relatedPort=parsed.relatedPort,
                            )
                            await self.pc.addIceCandidate(candidate)
                            self.logger.info(f"Added ICE candidate: {cand}")
            except Exception as e:
                self.logger.warning(f"WebSocket closed or errored: {e}")
            finally:
                self.logger.info("WebSocket loop exited.")
                ping_task.cancel()
                # Don't close the peer connection here


if __name__ == "__main__":
    client = WebRTCClientCamera()
    asyncio.run(client.connect())
