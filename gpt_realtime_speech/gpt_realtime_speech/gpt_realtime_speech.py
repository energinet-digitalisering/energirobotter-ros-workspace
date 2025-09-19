import asyncio
import base64
import json
import sounddevice as sd
import numpy as np
import websockets
import os

API_KEY = os.getenv("OPENAI_API_KEY")
URL = "wss://api.openai.com/v1/realtime?model=gpt-4o-realtime-preview"

# audio params
SAMPLE_RATE = 16000
BLOCK_SIZE = 512


async def audio_loop(ws):
    """Capture mic and send to GPT"""

    def callback(indata, frames, time, status):
        if status:
            print("Stream status:", status)
        pcm16 = (indata[:, 0] * 32767).astype(np.int16)
        b64 = base64.b64encode(pcm16.tobytes()).decode("utf-8")
        event = {"type": "input_audio_buffer.append", "audio": b64}
        asyncio.run_coroutine_threadsafe(
            ws.send(json.dumps(event)), asyncio.get_event_loop()
        )

    with sd.InputStream(
        channels=1, samplerate=SAMPLE_RATE, blocksize=BLOCK_SIZE, callback=callback
    ):
        while True:
            await asyncio.sleep(0.1)


async def speaker_loop(ws):
    """Play audio received from GPT"""
    async for msg in ws:
        data = json.loads(msg)
        if data.get("type") == "response.audio.delta":
            audio_b64 = data["delta"]
            pcm_bytes = base64.b64decode(audio_b64)
            pcm16 = (
                np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.float32) / 32767.0
            )
            sd.play(pcm16, SAMPLE_RATE)
        elif data.get("type") == "response.completed":
            print("Response done")


async def main():
    async with websockets.connect(
        URL,
        extra_headers={
            "Authorization": f"Bearer {API_KEY}",
            "OpenAI-Beta": "realtime=v1",
        },
    ) as ws:
        # start microphone task
        asyncio.create_task(audio_loop(ws))
        # request a response
        await ws.send(
            json.dumps(
                {
                    "type": "response.create",
                    "response": {"instructions": "You are connected. Speak now."},
                }
            )
        )
        # handle playback
        await speaker_loop(ws)


if __name__ == "__main__":
    asyncio.run(main())
