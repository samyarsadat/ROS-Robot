#  The ROS robot project (Pi Camera Server)
#  FastAPI HTTP server
#  Copyright 2024-2025 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, -2025.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https: www.gnu.org/licenses/>.

import asyncio
import logging
import uvicorn
from fastapi import FastAPI, Response, HTTPException, Depends
from starlette.responses import StreamingResponse, PlainTextResponse
from camera import Camera
from config import ServerConfig


# Main FastAPI application class
class Server:
    def __init__(self):
        self.logger = logging.getLogger("uvicorn.error")
        self.app = FastAPI()
        self.camera = Camera()

        self._video_stream_locks = [asyncio.Lock(), asyncio.Lock(), asyncio.Lock(), asyncio.Lock()]
        self._setup_routes()

    @staticmethod
    def single_client_dep(lock: asyncio.Lock):
        async def dependency():
            if not lock.locked():
                await lock.acquire()
            else:
                raise HTTPException(status_code=429, detail="Multiple connections on one endpoint not allowed!")
        return dependency

    def _setup_routes(self):
        @self.app.get("/video_stream/compressed/main",
                      dependencies=[Depends(self.single_client_dep(self._video_stream_locks[0]))],
                      response_class=StreamingResponse)
        async def hires_comp_stream():
            self.logger.info("Serving compressed hi-res stream.")
            self.camera.enable_stream(True, 0)

            return StreamingResponse(
                self.camera.server_video_stream_gen(self.camera.stream_outputs[0], "hires_comp", self._video_stream_locks[0]),
                media_type="multipart/x-mixed-replace; boundary=frame"
            )

        @self.app.get("/video_stream/compressed/lores",
                      dependencies=[Depends(self.single_client_dep(self._video_stream_locks[1]))],
                      response_class=StreamingResponse)
        async def lores_comp_stream():
            self.logger.info("Serving compressed lo-res stream.")
            self.camera.enable_stream(True, 1)

            return StreamingResponse(
                self.camera.server_video_stream_gen(self.camera.stream_outputs[1], "lores_comp", self._video_stream_locks[1]),
                media_type="multipart/x-mixed-replace; boundary=frame"
            )

        @self.app.get("/video_stream/uncompressed/main",
                      dependencies=[Depends(self.single_client_dep(self._video_stream_locks[2]))],
                      response_class=StreamingResponse)
        async def hires_stream():
            self.logger.info("Serving uncompressed hi-res stream.")
            self.camera.enable_stream(True, 2)

            return StreamingResponse(
                self.camera.server_video_stream_gen(self.camera.stream_outputs[2], "hires", self._video_stream_locks[2]),
                media_type="multipart/x-mixed-replace; boundary=frame"
            )

        @self.app.get("/video_stream/uncompressed/lores",
                      dependencies=[Depends(self.single_client_dep(self._video_stream_locks[3]))],
                      response_class=StreamingResponse)
        async def lores_stream():
            self.logger.info("Serving uncompressed lo-res stream.")
            self.camera.enable_stream(True, 3)

            return StreamingResponse(
                self.camera.server_video_stream_gen(self.camera.stream_outputs[3], "lores", self._video_stream_locks[3]),
                media_type="multipart/x-mixed-replace; boundary=frame"
            )

        @self.app.get("/test",
                      response_class=PlainTextResponse)
        def test_endpoint():
            return Response("TEST ENDPOINT")


# Run the server
if __name__ == "__main__":
    server_instance = None

    try:
        server_instance = Server()
        uvicorn.run(server_instance.app, host=ServerConfig.server_host, port=ServerConfig.server_port)
        server_instance.logger.info(f"Starting FastAPI server at http://{ServerConfig.server_host}:{ServerConfig.server_port}")
    except KeyboardInterrupt:
        if server_instance:
            server_instance.logger.info("Stopping the server.")
    finally:
        if server_instance:
            del server_instance
