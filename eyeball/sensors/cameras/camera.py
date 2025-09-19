import logging
import os
import threading
import time
from dataclasses import dataclass
from functools import wraps
from typing import Any, Dict, List, Optional, Protocol, Tuple, TypeVar, Union

import cv2
import numpy as np
from dotdict import dotdict
from eyeball.utils.utils import RateRecorder

# from eyeball.sensors.cameras.utils import async_complie_jpgs_to_mp4
from eyeball.data.data_utils import flatten_dict
from eyeball.utils.paramiko_utils import ServerConfig
from eyeball.utils.portal_utils import remote
from eyeball.utils.video_utils import StreamingVideoWriter

T = TypeVar("T")

def check_camera_exception(func):
    """Decorator to check for camera exceptions before executing methods.

    This decorator checks if self.exception is not None and raises it if so.
    This ensures that any camera polling errors are propagated to all method calls.
    """

    @wraps(func)
    def wrapper(self, *args, **kwargs):
        # Check if self.exception is not None and raise it
        if hasattr(self, "exception") and self.exception is not None:
            raise self.exception
        return func(self, *args, **kwargs)

    return wrapper


def nest_dotdict(d: Union[Dict[str, Any], T]) -> Union[dotdict, T]:
    """Convert nested dictionary to dotdict."""
    if isinstance(d, dict):
        d_dotdict = dotdict(d)
        for k, v in d_dotdict.items():
            if isinstance(v, dict):
                d_dotdict[k] = nest_dotdict(v)
        return d_dotdict
    return d


def plot_camera_read(camera: Any, save_datastream: bool = False, vis: bool = True) -> None:
    import cv2

    camera_data = camera.read()
    if vis:
        for k in camera_data.images.keys():
            cv2.namedWindow(k)

    counter = 0
    if not os.path.exists("images"):
        os.makedirs("images")
    if save_datastream and not os.path.exists("stream"):
        os.makedirs("stream")

    while True:
        data = camera.read()
        ts = data.timestamp
        camera_data = data.images

        key = cv2.waitKey(1)
        if vis:
            for k in camera_data.keys():
                # make the image array contiguous
                img = np.ascontiguousarray(camera_data[k])
                # plot ts on the image
                cv2.putText(
                    img,
                    f"ts in seconds: {ts / 1000}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (255, 0, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow(k, img[..., ::-1])
        if key == ord("s"):
            for k in camera_data.keys():
                cv2.imwrite(f"stream/{k}_{counter}.png", camera_data[k])
        if save_datastream:
            for k in camera_data.keys():
                cv2.imwrite(f"stream/{k}_{counter}.png", camera_data[k])
        counter += 1
        if key == 27:
            break


@dataclass
class IMUData:
    timestamp: float  # relative timestamp in ms
    acceleration: Optional[Tuple[float, float, float]] = None  # 3D acceleration [x, y, z]
    gyroscope: Optional[Tuple[float, float, float]] = None  # 3D gyroscope [x, y, z]


@dataclass
class CameraSpec:
    name: str  # Name of the camera
    shape: Tuple[int, int, int]  # Shape of the image (height, width, channels)
    dtype: type  # Data type of the image


@dataclass
class CameraData:
    images: Dict[str, Optional[np.ndarray]]  # Named dict of multiple arrays
    timestamp: float  # milliseconds unit
    imu_data: Optional[IMUData] = None  # Optional IMU data
    other_sensors: Optional[dict] = None  # Optional dictionary for additional sensors
    depth_data: Optional[np.ndarray] = None


class CameraDriver(Protocol):
    """Camera protocol.

    A protocol for a camera driver. This is used to abstract the camera from the rest of the code.
    """

    def read(self) -> CameraData:
        """Read a frame(RGB) from the camera.

        Returns:
            CameraData: The data read from the camera.
        """
        ...

    def read_calibration_data_intrinsics(self) -> Dict[str, Any]:
        """Read calibration data from the camera.

        Returns:
            IntrinsicsList: The calibration data.
        """
        ...

    def get_camera_info(self) -> Dict[str, Any]:
        """Retrieve camera information including device ID, resolution, FPS, and exposure settings.

        Returns:
            CameraInfoInternal: The camera information.
        """
        ...

    def stop(self) -> None:
        """Stop the camera."""
        ...


@dataclass
class CameraNode:
    camera: CameraDriver
    name: str
    timeout_sec: float = 0.2  # configurable timeout
    start_polling: bool = True
    remote_server_config: Optional[ServerConfig] = None

    def __post_init__(self):
        self._latest_data: Optional[CameraData] = None
        self.last_update_time: float = time.time()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.polling_thread = threading.Thread(target=self._poll_image, daemon=True)
        if self.start_polling:
            self.polling_thread.start()
        self.exception: Optional[Exception] = None
        while self._latest_data is None:
            time.sleep(0.1)
        self.stream_video_writer = None
        self.local_save_dir: Optional[str] = None
        self.async_saving_flag: bool | None = (
            None  # If None, it is not async mode. If True, it is async saving mode and it is recording. If False, it is async saving mode and it is not recording.
        )
        # Add a variable to track the last warning log time
        self.last_warning_log_time: float = 0.0

    def _poll_once(self) -> None:
        latest_data = self.camera.read()
        with self.lock:
            self._latest_data = latest_data
            self.last_update_time = time.time()

    def _poll_image(self) -> None:
        with RateRecorder(name=self.name) as rate_recorder:
            while not self.stop_event.is_set():
                try:
                    rate_recorder.track()
                    latest_data = self.camera.read()
                    with self.lock:
                        self._latest_data = latest_data
                        self.last_update_time = time.time()
                    # this is very important, else will cause busy waiting.
                    time.sleep(0.005)
                except Exception as e:
                    # together with check_camera_exception decorator this exception will be raised on the subsequence function calls.
                    # [ROB-571] do not raise this error, we will check timeout in `_get_latest_data`
                    # self.exception = e
                    # [SWE-156] this is very important, else will cause busy waiting.
                    time.sleep(0.005)
                    logging.error(f"Error polling server: {e}")

    @check_camera_exception
    @remote()
    def prepare_stream_video_writer(self, input_dict: Dict[str, Any]) -> None:
        self.async_saving_flag = True
        self.local_save_dir = input_dict["save_dir"]
        assert isinstance(self.local_save_dir, str), "local_save_dir should be a string"

        with self.lock:
            data = self._get_latest_data(image_as_contiguous_array=False)
        flattened_data = flatten_dict(data, sep="-")
        streamers = {}
        for k, _v in flattened_data.items():
            if "rgb" in k:
                image_save_path = os.path.join(self.local_save_dir, f"{self.name}-{k}.mp4")
                # temporarily use 30 fps, later will change it to the actual fps
                streamers[k] = StreamingVideoWriter(image_save_path, fps=30, crf=18)
                streamers[k].start_writing()
        self.stream_video_writer = streamers

    @check_camera_exception
    @remote()
    def add_frame(self) -> Dict[str, Any]:
        if self.stream_video_writer is None:
            logging.warning(f"Stream video writer not initialized for {self.name}")
            return {"result": "no stream video writer initialized"}
        with self.lock:
            data = self._get_latest_data(image_as_contiguous_array=False)
        flattened_data = flatten_dict(data, sep="-")
        timestamp = flattened_data.pop("timestamp")
        for k, v in flattened_data.items():
            self.stream_video_writer[k].add_frame(v)
        return {"result": "success", "timestamp": timestamp}

    @check_camera_exception
    @remote()
    def close_stream_video_writer(self, input_dict: Dict[str, Any]) -> Dict[str, Any]:
        self.async_saving_flag = False
        logging.info(f"Closing stream video writer for {self.name}")
        target_fps = input_dict.get("target_fps", None)
        blocked = input_dict.get("blocked", True)
        if self.stream_video_writer is None:
            logging.warning(
                f"Calling {self.name}.close_stream_video_writer: Stream video writer not initialized for {self.name}"
            )
            return {"result": "no stream video writer initialized"}
        existing_writer = self.stream_video_writer
        self.stream_video_writer = None

        def close():
            for k in existing_writer.keys():
                existing_writer[k].close(target_fps=target_fps)

        close_thread = threading.Thread(target=close)
        close_thread.start()
        if not blocked and self.remote_server_config is not None:
            blocked = True
            logging.warning(
                "when self.remote_server_config is not None, close_stream_video_writer must be in block mode"
            )
        if blocked:
            close_thread.join()
        if self.remote_server_config is not None:
            self._sync_to_remote()
        return {"result": "success"}

    @check_camera_exception
    @remote()
    def read(self) -> Dict[str, Any]:
        with self.lock:
            if self.last_update_time is None:
                raise RuntimeError("No data received yet")

            return self._get_latest_data(image_as_contiguous_array=True)

    def _get_latest_data(self, image_as_contiguous_array: bool = True) -> Dict[str, Any]:
        assert self._latest_data is not None, "latest_data should not be None at this point"
        if not self.start_polling:
            self._poll_once()

        # make sure the timeout is not exceeded
        current_time = time.time()
        time_diff = current_time - self.last_update_time
        if time_diff > self.timeout_sec:
            # when async saving mode, do not raise Error when it is not recording.
            if self.async_saving_flag is not None and self.async_saving_flag == False:  # noqa: E712
                # Only log warning every 5 seconds to avoid spam
                if current_time - self.last_warning_log_time >= 5.0:
                    logging.warning(
                        f"Camera Node {self.name}: camera read interval is {time_diff} seconds, bigger than expected {self.timeout_sec} seconds. This can be ignored when async saving mode is enabled and recording is not started."
                    )
                    self.last_warning_log_time = current_time
            else:
                self.exception = TimeoutError(
                    f"Camera Node {self.name}: camera read interval is {time_diff} seconds, bigger than expected {self.timeout_sec} seconds."
                )
                raise self.exception

        if not image_as_contiguous_array:
            return dict(
                images=self._latest_data.images,
                timestamp=self._latest_data.timestamp,
            )
        else:
            return dict(
                images={k: np.ascontiguousarray(v) for k, v in self._latest_data.images.items()},
                timestamp=self._latest_data.timestamp,
            )

    @check_camera_exception
    @remote(serialization_needed=True)
    def get_camera_info(self) -> Dict[str, Any]:
        return self.camera.get_camera_info() #.model_dump()

    @check_camera_exception
    @remote()
    def close(self) -> None:
        logging.info(f"Closing camera {self.name}")
        self.stop_event.set()
        self.camera.stop()
        self.polling_thread.join(timeout=1)

    