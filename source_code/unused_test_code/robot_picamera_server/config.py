#  The ROS robot project (Pi Camera Server)
#  Program configuration
#  Copyright 2024-2025 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2024-2025.
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

from libcamera import Transform


# FastAPI application config
class ServerConfig:
    server_host = "0.0.0.0"
    server_port = 8080


# Camera config (defaults)
class CameraConfig:
    default_camera_controls = {}
    default_camera_transform = Transform(vflip=True)
    default_stream_sizes_video = {"main": (1280, 720), "lores": (640, 480)}
    default_stream_sizes_still = {}
    default_stream_formats_video = {"main": "BGR888", "lores": "BGR888"}
    default_stream_formats_still = {}
    default_png_compress_lvl = 1   # 0-9
    default_jpg_quality = 95       # 0-95
    default_video_fps = 15
