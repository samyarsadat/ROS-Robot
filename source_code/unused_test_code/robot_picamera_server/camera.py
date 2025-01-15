#  The ROS robot project (Pi Camera Server)
#  PiCamera2 camera utils
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

import asyncio
import io
import logging
import cv2
import numpy as np
from threading import RLock, Lock, Condition
from typing import Union, Generator
from PIL import Image
from libcamera import Transform
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder, Encoder
from picamera2.outputs import FileOutput
from config import CameraConfig


# Camera handling class (singleton)
# Note: Camera will always stay in video configuration when outside the scope
#       of the capture_still() and get_control_limits(config_type="still") methods.
#       This is ensured by the camera access lock (_cam_lock). As a result of this,
#       any methods which relly upon this guarantee will not function properly if
#       called from capture_still() or get_control_limits(config_type="still").
class Camera:
    class StreamOutput(io.BufferedIOBase):
        def __init__(self):
            self.frame = None
            self.condition = Condition()

        def write(self, buf) -> None:
            with self.condition:
                self.frame = buf
                self.condition.notify_all()

    class _CameraConfig:
        class _StreamConfig:
            img_format: Union[str, None]
            size: Union[tuple[int, int], None]

            def __init__(self, img_format: Union[str, None], size: Union[tuple[int, int], None]):
                self.img_format = img_format
                self.size = size

            def as_dict(self) -> dict:
                ret_dict = {}
                if self.img_format: ret_dict["format"] = self.img_format
                if self.size: ret_dict["size"] = self.size
                return ret_dict

        class _SensorConfig:
            output_size: Union[tuple[int, int], None]
            bit_depth: Union[int, None]

            def __init__(self, output_size: Union[tuple[int, int], None], bit_depth: Union[int, None]):
                self.output_size = output_size
                self.bit_depth = bit_depth

            def as_dict(self) -> dict:
                ret_dict = {}
                if self.output_size: ret_dict["output_size"] = self.output_size
                if self.bit_depth: ret_dict["bit_depth"] = self.bit_depth
                return ret_dict

        def __init__(self, cam_lock: RLock, rpi_cam: Picamera2, streams: list[str], config_type: str):
            self._rpi_cam = rpi_cam
            self._cam_lock = cam_lock
            self._config_type = config_type
            self._align_config = False
            self.set_camera_conf = False
            self._config = None

            match config_type:
                case "video":
                    self._config = self._rpi_cam.create_video_configuration()
                case "still":
                    self._config = self._rpi_cam.create_still_configuration()
                case _:
                    raise RuntimeError(f"Unknown camera config type: {config_type}")

            self._stream_confs = {}
            self._transform = None
            self._colour_space = None
            self._sensor = None
            self._controls = {}

            for stream in streams:
                if stream in ["main", "lores", "raw"]:
                    self._stream_confs[stream] = (self._StreamConfig(None, None))

        def _recreate_config(self) -> None:
            match self._config_type:
                case "video":
                    self._config = self._rpi_cam.create_video_configuration(
                        main=self._stream_confs["main"].as_dict() if "main" in self._stream_confs else {},
                        lores=self._stream_confs["lores"].as_dict() if "lores" in self._stream_confs else None,
                        raw=self._stream_confs["raw"].as_dict() if "raw" in self._stream_confs else {},
                        transform=self._transform if self._transform else Transform(),
                        colour_space=self._colour_space,
                        sensor=self._sensor.as_dict() if self._sensor else {},
                        controls=self._controls
                    )
                case "still":
                    self._config = self._rpi_cam.create_still_configuration(
                        main=self._stream_confs["main"].as_dict() if "main" in self._stream_confs else {},
                        lores=self._stream_confs["lores"].as_dict() if "lores" in self._stream_confs else None,
                        raw=self._stream_confs["raw"].as_dict() if "raw" in self._stream_confs else {},
                        transform=self._transform if self._transform else Transform(),
                        colour_space=self._colour_space,
                        sensor=self._sensor.as_dict() if self._sensor else {},
                        controls=self._controls
                    )

            if self._align_config:
                self._rpi_cam.align_configuration(self._config)

            if self.set_camera_conf:
                with self._cam_lock:
                    self._rpi_cam.switch_mode(self._config)

        @property
        def config(self) -> dict:
            return self._config

        @property
        def align_config(self) -> bool:
            return self._align_config

        @align_config.setter
        def align_config(self, align_config: bool):
            if align_config != self._align_config:
                self._align_config = align_config
                self._recreate_config()

        def align_config_once(self) -> None:
            self._rpi_cam.align_configuration(self._config)

        @property
        def stream_sizes(self) -> dict:
            sizes = {}

            for key, value in self._stream_confs.items():
                sizes[key] = value.size

            return sizes

        @stream_sizes.setter
        def stream_sizes(self, sizes: dict) -> None:
            if isinstance(sizes, dict):
                for key, value in sizes.items():
                    if key in self._stream_confs and isinstance(value, (list, tuple)) and len(value) == 2:
                        self._stream_confs[key].size = (int(value[0]), int(value[1]))
                self._recreate_config()

        @property
        def stream_formats(self) -> dict:
            formats = {}

            for key, value in self._stream_confs.items():
                formats[key] = value.img_format

            return formats

        @stream_formats.setter
        def stream_formats(self, formats: dict) -> None:
            if isinstance(formats, dict):
                for key, value in formats.items():
                    if key in self._stream_confs and isinstance(value, str):
                        self._stream_confs[key].img_format = value
                self._recreate_config()

        @property
        def sensor(self) -> dict:
            return self._sensor.__dict__ if self._sensor else {}

        @sensor.setter
        def sensor(self, sensor: dict) -> None:
            if isinstance(sensor, dict):
                if not sensor:
                    self._sensor = None
                    return

                output_size = None
                bit_depth = None

                for key, value in sensor.items():
                    if key == "output_size" and isinstance(value, (list, tuple)) and len(value) == 2:
                        output_size = (int(value[0]), int(value[1]))
                    elif key == "bit_depth" and isinstance(value, (int, float)):
                        bit_depth = int(value)

                if bit_depth and output_size:
                    self._sensor = self._SensorConfig(output_size, bit_depth)
                self._recreate_config()

        @property
        def colour_space(self) -> Union[str, None]:
            return self._colour_space

        @colour_space.setter
        def colour_space(self, colour_space: str) -> None:
            if not colour_space:
                self._colour_space = None
            elif isinstance(colour_space, str):
                self._colour_space = colour_space
            self._recreate_config()

        @property
        def transform(self) -> Union[Transform, None]:
            return self._transform

        @transform.setter
        def transform(self, transform: Transform) -> None:
            if not transform:
                self._transform = None
            elif isinstance(transform, Transform):
                self._transform = transform
            self._recreate_config()

        @property
        def controls(self) -> dict:
            return self._controls

        @controls.setter
        def controls(self, controls: dict) -> None:
            if not controls:
                self._controls = {}
            elif isinstance(controls, dict):
                for key, value in controls.items():
                    if not value:
                        del self._controls[key]
                    else:
                        self._controls[key] = value
            self._recreate_config()

    def __init__(self):
        self._logger = logging.getLogger("uvicorn.error")
        self._logger.debug("Initializing camera class...")

        self._rpi_cam = Picamera2()
        self._cam_lock = RLock()

        self._encoders = [MJPEGEncoder(), MJPEGEncoder(), Encoder(), Encoder()]   # Main (compressed), lores (compressed), main, lores
        self.stream_outputs = [Camera.StreamOutput(), Camera.StreamOutput(),
                               Camera.StreamOutput(), Camera.StreamOutput()]      # Main (compressed), lores (compressed), main, lores
        self._stream_names = ["main", "lores", "main", "lores"]                   # Main (compressed), lores (compressed), main, lores

        self.video_config = self._CameraConfig(self._cam_lock, self._rpi_cam, ["main", "lores"], "video")
        self.still_config = self._CameraConfig(self._cam_lock, self._rpi_cam, ["main"], "still")

        self._video_stream_states = [False, False, False, False]   # Main (hires), secondary (lores), main, lores

        self.video_config.stream_sizes = CameraConfig.default_stream_sizes_video
        self.still_config.stream_sizes = CameraConfig.default_stream_sizes_still
        self.video_config.stream_formats = CameraConfig.default_stream_formats_video
        self.still_config.stream_formats = CameraConfig.default_stream_formats_still
        self.video_config.controls = CameraConfig.default_camera_controls
        self.still_config.controls = CameraConfig.default_camera_controls
        self.video_config.transform = CameraConfig.default_camera_transform
        self.still_config.transform = CameraConfig.default_camera_transform
        self.video_fps = CameraConfig.default_video_fps
        self.still_quality = (CameraConfig.default_png_compress_lvl, CameraConfig.default_jpg_quality)
        self.video_config.align_config_once()
        self.still_config.align_config_once()
        self._rpi_cam.configure(self.video_config.config)
        self.video_config.set_camera_conf = True

        self._logger.info("Camera class initialized. Starting camera.")
        self._rpi_cam.start()

    def __del__(self):
        with self._cam_lock:
            self._logger.info("Stopping video streams and stopping camera.")

            for i in range(0, len(self._video_stream_states)):
                self.enable_stream(False, i)

            self._rpi_cam.stop()
            self._rpi_cam.close()

    def _mode_to_still(self) -> list[bool]:
        prev_stream_states = self._video_stream_states

        for i in range(0, len(self._video_stream_states)):
            self.enable_stream(False, i)

        self._rpi_cam.switch_mode(self.still_config.config)
        return prev_stream_states

    def _mode_to_video(self, stream_states: list[bool]):
        self._rpi_cam.switch_mode(self.video_config.config)

        for i in range(0, len(self._video_stream_states)):
            self.enable_stream(stream_states[i], i)

    def enable_stream(self, enable: bool, index: int) -> None:
        with self._cam_lock:
            if enable and not self._video_stream_states[index]:
                self._video_stream_states[index] = True
                self._rpi_cam.start_encoder(self._encoders[index], FileOutput(self.stream_outputs[index]), name=self._stream_names[index])
                self._logger.debug(f"Enabled camera stream no. {index}.")
            elif not enable and self._video_stream_states[index]:
                self._video_stream_states[index] = False
                self._rpi_cam.stop_encoder(self._encoders[index])
                self._logger.debug(f"Disabled camera stream no. {index}.")

    # Note: will return config when in video config!
    @property
    def current_config(self) -> dict:
        with self._cam_lock:
            return self._rpi_cam.camera_configuration()

    def get_control_limits(self, config_type: str) -> dict:
        with self._cam_lock:
            if config_type == "still":
                prev_stream_states = self._mode_to_still()

            gain_min, gain_max, gain_default = self._rpi_cam.camera_controls["AnalogueGain"]
            exp_t_min, exp_t_max, exp_t_default = self._rpi_cam.camera_controls["ExposureTime"]
            frame_d_min, frame_d_max, frame_d_default = self._rpi_cam.camera_controls["FrameDurationLimits"]
            sclr_c_min, sclr_c_max, sclr_c_default = self._rpi_cam.camera_controls["ScalerCrop"]

            if config_type == "still":
                self._mode_to_video(prev_stream_states)

        limits = {
            "AnalogueGain": {
                "min": gain_min,
                "max": gain_max,
                "default": gain_default
            },

            "ExposureTime": {
                "min": exp_t_min,
                "max": exp_t_max,
                "default": exp_t_default
            },

            "FrameDurationLimits": {
                "min": frame_d_min,
                "max": frame_d_max,
                "default": frame_d_default
            },

            "ScalerCrop": {
                "min": sclr_c_min,
                "max": sclr_c_max,
                "default": sclr_c_default
            }
        }

        return limits

    # Note: will return controls when in video config!
    @property
    def current_controls(self) -> dict:
        with self._cam_lock:
            return self._rpi_cam.controls.make_dict()

    def cycle_autofocus(self) -> None:
        self._rpi_cam.autofocus_cycle()

    @property
    def video_fps(self) -> float:
        return 1e6 / self.current_controls["FrameDurationLimits"][0]

    @video_fps.setter
    def video_fps(self, fps: int) -> None:
        with self._cam_lock:
            frame_duration_lims = self.get_control_limits("video")["FrameDurationLimits"]
            frame_duration = int(1e6 / fps)

            if frame_duration > frame_duration_lims["max"]:
                self._logger.info("Unable to set desired frame duration (above max limit).")
                frame_duration = frame_duration_lims["max"]
            elif frame_duration < frame_duration_lims["min"]:
                self._logger.info("Unable to set desired frame duration (below min limit).")
                frame_duration = frame_duration_lims["min"]

            self.video_config.controls = {"FrameDurationLimits": (frame_duration, frame_duration)}

    @property
    def still_quality(self) -> tuple[int, int]:
        return self._rpi_cam.options["compress_level"], self._rpi_cam.options["quality"]

    @still_quality.setter
    def still_quality(self, quality_params: tuple) -> None:
        self._rpi_cam.options["compress_level"] = quality_params[0]   # PNG
        self._rpi_cam.options["quality"] = quality_params[1]          # JPEG

    @property
    def camera_properties(self) -> dict:
        return self._rpi_cam.camera_properties

    @property
    def sensor_modes(self) -> list[dict]:
        return self._rpi_cam.sensor_modes

    def capture_metadata(self) -> dict:
        with self._cam_lock:
            return self._rpi_cam.capture_metadata()

    def capture_still(self, img_format: str, cycle_autofocus: bool = False) -> tuple[bytes, dict]:
        with self._cam_lock:
            self._logger.debug(f"Disabling video encoders to capture still.")
            prev_stream_states = self._mode_to_still()

            if cycle_autofocus:
                self.cycle_autofocus()

            image_data = io.BytesIO()
            metadata = self._rpi_cam.capture_file(image_data, name="main", format=img_format)

            self._mode_to_video(prev_stream_states)
            self._logger.debug(f"Capture complete. Camera set back to video config.")

            return image_data.getvalue(), metadata

    async def server_video_stream_gen(self, output: StreamOutput, stream_name: str, stream_lock: Lock) -> Generator[bytes, None, None]:
        stream_index = 0

        try:
            match stream_name:
                case "hires_comp":
                    stream_index = 0
                    picam_stream_name = "main"
                case "lores_comp":
                    stream_index = 1
                    picam_stream_name = "lores"
                case "hires":
                    stream_index = 2
                    picam_stream_name = "main"
                case "lores":
                    stream_index = 3
                    picam_stream_name = "lores"
                case _:
                    raise asyncio.CancelledError()
                    
            stream_format = self.current_config[picam_stream_name]["format"]

            while True:
                with output.condition:
                    output.condition.wait()
                    if not stream_name.endswith("_comp"):
                        try:
                            match stream_format:
                                case "YUV420":
                                    img = Image.fromarray(cv2.cvtColor(np.frombuffer(output.frame, dtype=np.uint8).reshape(
                                                                        (self._encoders[stream_index].size[1] * 3) // 2,
                                                                        self._encoders[stream_index].size[0]), cv2.COLOR_YUV2RGB_I420), 
                                                                        "RGB")
                                case "RGB888":
                                    img = Image.fromarray(np.frombuffer(output.frame, dtype=np.uint8).reshape(
                                                                        self._encoders[stream_index].size[1], 
                                                                        self._encoders[stream_index].size[0], 3)[:, :, ::-1], 
                                                                        "RGB")
                                case "BGR888":
                                    output.frame.seek(0)
                                    img = Image.frombytes("RGB", self._encoders[stream_index].size, output.frame.read())
                                case "XRGB8888":
                                    img = Image.fromarray(np.frombuffer(output.frame, dtype=np.uint8).reshape(
                                                                        self._encoders[stream_index].size[1], 
                                                                        self._encoders[stream_index].size[0], 4)[:, :, :3][:, :, ::-1], 
                                                                        "RGB")
                                case "XBGR8888":
                                    img = Image.fromarray(np.frombuffer(output.frame, dtype=np.uint8).reshape(
                                                                        self._encoders[stream_index].size[1], 
                                                                        self._encoders[stream_index].size[0], 4)[:, :, :3], 
                                                                        "RGB")

                            byte_stream = io.BytesIO()
                            img.save(byte_stream, format="BMP")
                            byte_stream.seek(0)
                            yield b"--frame\r\nContent-Type: image/bmp\r\n\r\n" + byte_stream.getvalue() + b"\r\n"
                        except Exception as e:
                            stream_format = self.current_config[picam_stream_name]["format"]
                            self._logger.error("Error during server frame preparation: " + str(e))
                    else:
                        yield b"--frame\r\nContent-Type: image/jpg\r\n\r\n" + output.frame + b"\r\n"
                await asyncio.sleep(0)
        except asyncio.CancelledError:
            self._logger.info(f"Client disconnected. Stopping encoder for [{stream_name}].")
            self.enable_stream(False, stream_index)
            stream_lock.release()
