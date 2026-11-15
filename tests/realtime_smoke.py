"""Exercise two WAV turns over one WebSocket, using only synthetic PCM."""

import argparse
import asyncio
import base64
import json
from pathlib import Path
import struct
import tempfile
from urllib.parse import parse_qs, urlsplit
import wave

from websockets.asyncio.server import serve


def write_input(path, rate, frame):
    with wave.open(str(path), "wb") as wav:
        wav.setparams((len(frame), 2, rate, 0, "NONE", "not compressed"))
        wav.writeframes(struct.pack("<" + "h" * len(frame), *frame) * (rate // 5))


async def smoke(client, legacy_echo=False):
    connections = 0
    fixture_done = asyncio.get_running_loop().create_future()
    replies = [struct.pack("<h", sample) * 513 for sample in (301, -302)]
    transcripts = ["你好，蓝莓！", "你刚才说你叫蓝莓。"]

    async def handle(ws):
        nonlocal connections
        connections += 1
        try:
            assert parse_qs(urlsplit(ws.request.path).query) == {
                "model": ["qwen3.8-omni-flash-realtime"], "test": ["retained"],
            }
            await ws.send(json.dumps({"type": "session.created", "session": {"id": "test"}}))
            config = json.loads(await ws.recv())
            assert config["type"] == "session.update", config
            session = config["session"]
            assert "audio" not in session
            assert session["modalities"] == ["text", "audio"]
            assert session["turn_detection"]["type"] == "server_vad"
            session["model"] = "qwen3.8-omni-flash-realtime"
            if legacy_echo:
                session["input_audio_format"] = "pcm"
                session["output_audio_format"] = "pcm"
            else:
                session["turn_detection"].pop("type")
            await ws.send(json.dumps({"type": "session.updated", "session": session}))
            for turn, expected_input in enumerate((1200, -800), 1):
                speech = False
                silence = 0
                while silence < 800 * 16:
                    event = json.loads(await ws.recv())
                    assert event["type"] == "input_audio_buffer.append", event["type"]
                    pcm = base64.b64decode(event["audio"], validate=True)
                    samples = struct.unpack("<" + "h" * (len(pcm) // 2), pcm)
                    assert 0 < len(samples) <= 320, len(samples)
                    if samples[-1] == expected_input and not speech:
                        speech = True
                        await ws.send(json.dumps({"type": "input_audio_buffer.speech_started"}))
                    if speech and not any(samples):
                        silence += len(samples)
                response_id = f"test-{turn}"
                await ws.send(json.dumps({"type": "input_audio_buffer.speech_stopped"}))
                await ws.send(json.dumps({"type": "response.created", "response": {"id": response_id}}))
                pcm = replies[turn - 1]
                for chunk in (pcm[:514], pcm[514:]):
                    audio = json.dumps({
                        "type": "response.audio.delta",
                        "response_id": response_id,
                        "delta": base64.b64encode(chunk).decode("ascii"),
                    })
                    await ws.send([audio[:25], audio[25:]])
                await ws.send(json.dumps({
                    "type": "response.audio_transcript.done", "response_id": response_id,
                    "transcript": transcripts[turn - 1],
                }, ensure_ascii=False))
                await ws.send(json.dumps({
                    "type": "response.done",
                    "response": {"id": response_id, "status": "completed"},
                }))
            fixture_done.set_result(None)
            await ws.wait_closed()
        except Exception as error:
            if not fixture_done.done():
                fixture_done.set_exception(error)

    with tempfile.TemporaryDirectory(prefix="microduck-voice-") as directory:
        directory = Path(directory)
        first, second = directory / "one.wav", directory / "two.wav"
        write_input(first, 48000, (-9000, 1200))
        write_input(second, 16000, (-800,))
        output = directory / "replies"
        async with serve(handle, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            url = f"ws://127.0.0.1:{port}/?model=qwen3.5-omni-flash-realtime&test=retained"
            process = await asyncio.create_subprocess_exec(
                str(client), "--mock", url, "--out", str(output),
                "--wav", str(first), "--wav", str(second),
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
            )
            try:
                stdout, stderr = await asyncio.wait_for(process.communicate(), timeout=20)
                if fixture_done.done():
                    fixture_done.result()
                assert process.returncode == 0, stderr.decode(errors="replace")
                assert fixture_done.done() and connections == 1, connections
                assert b"Listening (2)" in stdout, stdout
                assert all(f"Microduck: {line}" in stdout.decode("utf-8")
                           for line in transcripts), stdout
            finally:
                if process.returncode is None:
                    process.kill()
                    await process.wait()
        for turn, expected in enumerate(replies, 1):
            with wave.open(str(output / f"reply-{turn}.wav"), "rb") as wav:
                assert (wav.getnchannels(), wav.getsampwidth(), wav.getframerate()) == (1, 2, 24000)
                assert wav.readframes(wav.getnframes()) == expected
    echo = "explicit PCM" if legacy_echo else "default PCM"
    print(f"PASS: {echo} session echo, two WAV turns on one WebSocket and exact PCM replies")


async def expect_error(client, event, expected, after_update=False):
    async def handle(ws):
        if after_update:
            await ws.send(json.dumps({"type": "session.created"}))
            assert json.loads(await ws.recv())["type"] == "session.update"
        await ws.send(json.dumps(event))
        await ws.wait_closed()

    with tempfile.TemporaryDirectory(prefix="microduck-voice-") as directory:
        async with serve(handle, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            process = await asyncio.create_subprocess_exec(
                str(client), "--mock", f"ws://127.0.0.1:{port}/",
                "--out", directory, "--wav", "unused.wav",
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
            )
            try:
                stdout, stderr = await asyncio.wait_for(process.communicate(), timeout=10)
                assert process.returncode == 1 and b"Listening" not in stdout, (stdout, stderr)
                diagnostic = stderr.decode()
                assert all(needle in diagnostic for needle in expected), diagnostic
            finally:
                if process.returncode is None:
                    process.kill()
                    await process.wait()


async def run(client):
    await smoke(client)
    await smoke(client, legacy_echo=True)
    await expect_error(client, {
        "request_id": "request-test",
        "code": "InvalidParameter",
        "message": "Unsupported model",
    }, ("session.created", "code=InvalidParameter", "message=Unsupported model", "request-test"))
    await expect_error(client, {
        "header": {
            "event": "task-failed", "task_id": "task-test",
            "error_code": "InvalidApiKey", "error_message": "Key was rejected",
        },
        "payload": {},
    }, ("session.created", "code=InvalidApiKey", "message=Key was rejected", "task-test"))
    await expect_error(client, {
        "type": "error", "event_id": "event-test",
        "error": {"code": "invalid_value", "message": "Invalid audio format", "param": "session.audio"},
    }, ("session.updated", "code=invalid_value", "Invalid audio format", "session.audio", "event-test"),
        after_update=True)
    await expect_error(client, {
        "type": "session.updated",
        "session": {
            "input_audio_format": "pcm", "output_audio_format": "pcm",
            "audio": {"input": {"format": {"type": "pcm", "sample_rate": 8000}}},
            "turn_detection": {"type": "server_vad"},
        },
    }, ("session.updated returned incompatible input audio, expected PCM at 16000 Hz",),
        after_update=True)
    await expect_error(client, {
        "type": "session.updated", "session": {"turn_detection": None},
    }, ("session.updated did not confirm turn_detection.type=server_vad",), after_update=True)
    print("PASS: flat and gateway error diagnostics, standard Realtime errors and incompatible PCM")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("client", type=Path)
    args = parser.parse_args()
    asyncio.run(run(args.client.resolve()))
