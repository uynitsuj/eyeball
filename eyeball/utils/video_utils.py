import json
import logging
import os
import queue
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, List, Optional

import cv2
import imageio
import numpy as np
from PIL import Image

Image.MAX_IMAGE_PIXELS = 400_000_000

# get number of cores
num_cores: Optional[int] = os.cpu_count()


def pad_video_freeze(video: np.ndarray, target_length: int) -> np.ndarray:
    """Pad a video to the target length by repeating the last frame."""
    last_frame = video[-1]
    add_frames = target_length - len(video)
    video = np.concatenate([video, np.repeat(last_frame[None], add_frames, axis=0)], axis=0)
    return video


def pad_video_repeat(video: np.ndarray, target_length: int) -> np.ndarray:
    """Pad a video to the target length by replaying the video."""
    # times to repeat
    times = target_length // len(video)
    remainder = target_length % len(video)
    video = np.concatenate([video] * times + [video[:remainder]], axis=0)
    return video


def repeat_video(video: np.ndarray, repeat_times: int) -> np.ndarray:
    """Repeat a video to the target length."""
    video = np.concatenate([video] * repeat_times, axis=0)
    return video


def decode_mp4_generator(video_path: str):
    """
    Efficiently load and decode MP4 file as a generator.

    Parameters:
    - video_path (str): Path to the MP4 file.

    Yields:
    - frame (np.ndarray): One frame at a time.
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise ValueError(f"Cannot open video file: {video_path}")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            yield frame[:, :, ::-1]
    finally:
        cap.release()


def save_jpgs_to_mp4_ffmpeg(images_dir: str, output_mp4: str, fps: float = 30.0, crf: int = 18):
    codec = "libx264"
    cmd = [
        "nice",
        "-n",
        "10",
        "ffmpeg",
        "-y",
        "-framerate",
        str(fps),  # set input frame rate
        "-pattern_type",
        "glob",  # allows glob wildcards for input
        "-i",
        "*.jpg",  # or use os.path.join(dir, "*.jpg")
        "-vf",
        "colorchannelmixer=rr=0:rb=1:gg=1:bb=0:br=1",
        "-an",
        "-c:v",
        codec,
        "-preset",
        "fast",
        "-profile:v",
        "baseline",
        "-level",
        "3.0",
        "-crf",
        str(crf),
        "-pix_fmt",
        "yuv420p",
        "-threads",
        "3",
        output_mp4,
    ]
    subprocess.run(cmd, cwd=images_dir, check=True)


class StreamingVideoWriter:
    """Thread-safe streaming video writer that writes frames to MP4 as they arrive."""

    def __init__(self, output_path: str, fps: int = 30, crf: int = 18):
        self.output_path = output_path
        self.fps = fps
        self.crf = crf
        self.frame_queue: queue.Queue = queue.Queue()
        self.stop_event = threading.Event()
        self.writer: Optional[Any] = None
        self.writer_thread: Optional[threading.Thread] = None
        self.frame_count = 0

        # Ensure output directory exists
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

    def start_writing(self) -> None:
        """Start the writing thread."""
        if self.writer_thread is None:
            self.writer_thread = threading.Thread(target=self._write_frames, daemon=True)
            self.writer_thread.start()

    def _init_writer(self, frame_shape: tuple) -> None:
        """Initialize video writer with the frame shape."""
        if num_cores is None:
            writing_threads = 2
        else:
            writing_threads = max(int(num_cores / 5), 1)
        self.writer = imageio.get_writer(
            self.output_path,
            mode="I",
            fps=self.fps,
            codec="libx264",
            ffmpeg_params=[
                "-threads",
                str(writing_threads),
                "-crf",
                str(self.crf),
                "-preset",
                "fast",
                "-profile:v",
                "baseline",
                "-level",
                "3.0",
                "-pix_fmt",
                "yuv420p",
            ],
        )

    def _write_frames(self) -> None:
        """Background thread that writes frames to video files."""
        try:
            while not self.stop_event.is_set():
                try:
                    # wait if frame queue has <= 1 frame
                    while self.frame_queue.qsize() <= 1:
                        time.sleep(0.1)
                        continue

                    # Get frame with timeout to allow checking stop_event
                    frame = self.frame_queue.get(timeout=0.1)

                    # Initialize writer if this is the first frame
                    if self.writer is None:
                        self._init_writer(frame.shape)
                    if self.writer is not None:
                        self.writer.append_data(np.ascontiguousarray(frame))

                    self.frame_count += 1
                    self.frame_queue.task_done()
                    time.sleep(0.01)
                except queue.Empty:
                    continue

        except Exception as e:
            logging.error(f"Error in streaming video writer: {e}")
        finally:
            # Process any remaining frames
            while not self.frame_queue.empty():
                try:
                    frame = self.frame_queue.get_nowait()
                    if self.writer is not None:
                        self.writer.append_data(np.ascontiguousarray(frame))
                    self.frame_count += 1
                    self.frame_queue.task_done()
                except queue.Empty:
                    break

    def add_frame(self, frame: np.ndarray) -> None:
        """Add a frame to be written to the video."""
        if not self.stop_event.is_set():
            self.frame_queue.put(frame)

    def close(self, target_fps: Optional[int] = None) -> None:
        """Close the video writer and stop the writing thread."""
        while not self.frame_queue.qsize() == 1:
            # make sure all frames are written except the last one
            time.sleep(0.1)

        self.stop_event.set()  # stop the writing thread when there is only 1 frame left

        # may not be necessary, but just in case there is memory leak
        self.frame_queue.get()  # remove the last frame from the queue

        logging.info(f"Closing streaming video writer for {self.output_path}")
        # Wait for all queued frames to be written
        if self.writer_thread is not None:
            self.writer_thread.join(timeout=2.0)

        # Close writer
        if self.writer is not None:
            try:
                self.writer.close()
            except Exception as e:
                logging.error(f"Error closing writer: {e}")

        logging.info(f"Streaming video writer closed. Wrote {self.frame_count} frames to {self.output_path}")
        # TODO: is this needed?
        # if target_fps is not None:
        #     change_video_fps(self.output_path, target_fps)
        #     change_video_fps(self.low_res_output_path, target_fps)
        #     logging.info(f"SteamingVideoWritter, converting {self.output_path} to {target_fps} fps")


def change_video_fps(video_path: str, target_fps: int) -> None:
    """
    This command just change the playback speed.
    ffmpeg -i input.mp4 -c copy -r 30 output.mp4
    """
    cmd = [
        "ffmpeg",
        "-i",
        video_path,
        "-c",
        "copy",
        "-r",
        str(target_fps),
        video_path,
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def compress_images_to_mp4(output_path: str, image_list: list, fps: int = 30, crf: int = 18) -> None:
    """
    Compress a list of images(RGB) into an MP4 video efficiently via FFmpeg pipe.

    Parameters:
    - output_path (str): Path to save the MP4 video.
    - image_list (list of np.ndarray): List of image frames (e.g., [H, W, 3]).
    - fps (int): Frames per second for the output video.
    - quality (int): 1 (worst) to 10 (best). Internally mapped to CRF.

    Returns:
    - None
    """
    codec = "libx264"
    height, width = image_list[0].shape[:2]
    cmd = [
        "ffmpeg",
        "-y",
        "-f",
        "rawvideo",
        "-vcodec",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{width}x{height}",
        "-r",
        str(fps),
        "-i",
        "-",
        "-an",
        "-c:v",
        codec,
        "-preset",
        "fast",
        "-profile:v",
        "baseline",
        "-level",
        "3.0",
        "-crf",
        str(crf),
        "-pix_fmt",
        "yuv420p",
        "-threads",
        "3",
        output_path,
    ]

    proc = subprocess.Popen(
        cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    assert proc.stdin is not None, "FFmpeg process stdin is closed."
    for img in image_list:
        proc.stdin.write(img.tobytes())
    proc.stdin.close()
    proc.wait()


def compress_images_to_mp4_path(input_path: str, output_path: str, crf: int = 18) -> None:
    """
    Compress an existing video file using FFmpeg.

    Parameters:
    - input_path (str): Path to the original video file.
    - output_path (str): Path to save the compressed MP4 video.
    - crf (int): Constant Rate Factor (lower = better quality, 18-28 typical).

    Returns:
    - None
    """
    codec = "libx264"
    cmd = [
        "ffmpeg",
        "-y",
        "-i",
        input_path,
        "-an",
        "-c:v",
        codec,
        "-preset",
        "fast",
        "-profile:v",
        "baseline",
        "-level",
        "3.0",
        "-crf",
        str(crf),
        "-pix_fmt",
        "yuv420p",
        "-threads",
        "16",
        output_path,
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def compress_images_to_mp4_imageio(
    output_path: str, image_list: list, fps: int = 30, crf: int = 18, codec: str = "libx264"
) -> None:
    """
    Compress a list of images into an MP4 video efficiently.

    Parameters:
    - output_path (str): Path to save the MP4 video.
    - image_list (list of np.ndarray): List of image frames (e.g., [H, W, 3]).
    - fps (int): Frames per second for the output video.
    - crf (int): Quality of the video (lower is higher quality, 18 is usually good).

    crf0: 20mb/min
    crf10: 5mb/min
    crf17: 2mb/min

    Returns:
    - None
    """
    params = ["-threads", "8", "-crf", str(crf)]
    if codec == "libx265":
        params += ["-tag:v", "hvc1"]

    # Use extension-based approach to avoid format enum issue
    writer = imageio.get_writer(  # type: ignore
        output_path,
        mode="I",
        fps=fps,
        codec=codec,
        ffmpeg_params=params,
    )

    try:
        for image in image_list:
            writer.append_data(np.ascontiguousarray(image))
    finally:
        writer.close()


def get_specific_frames_pyav(video_path, frame_indices, percentage=False):
    """
    Efficiently fetch specific frames from an MP4 file using PyAV while preserving the order of queried indices.

    Parameters:
    - video_path (str): Path to the MP4 file.
    - frame_indices (list of int): List of frame indices to fetch (0-based).
    - percentage (bool) :interpret frame_indices as percentage instead

    Returns:
    - frames (list of np.ndarray): List of the decoded frames as NumPy arrays, in the same order as the input indices.
    """
    import av

    container = av.open(video_path)
    stream = container.streams.video[0]

    # Get the total number of frames in the video
    total_frames = stream.frames

    if percentage:
        frame_indices = [int(fraction * (total_frames - 1)) for fraction in frame_indices]

    if total_frames is None:
        raise ValueError("Cannot determine the total frame count.")

    if any(idx < 0 or idx >= total_frames for idx in frame_indices):
        raise IndexError(f"Some frame indices are out of range (0-{total_frames - 1}).")

    # Sort indices to reduce seeking overhead
    index_to_order = {idx: i for i, idx in enumerate(frame_indices)}
    unique_sorted_indices = sorted(set(frame_indices))

    frames = []
    decoder = container.decode(stream)

    # Read and decode only the necessary frames
    current_frame_idx = 0
    decoded_frames = {}

    for frame in decoder:
        if current_frame_idx in unique_sorted_indices:
            # Convert the frame to an RGB image
            img = frame.to_image()
            decoded_frames[current_frame_idx] = np.array(img)

            if len(decoded_frames) == len(unique_sorted_indices):
                break

        current_frame_idx += 1

    container.close()

    # Reorder frames to match the input indices
    frames = [decoded_frames[idx] for idx in frame_indices]
    return np.array(frames)


def get_specific_frames(video_path, frame_indices, percentage=False):
    """
    Efficiently fetch specific frames from an MP4 file while preserving the order of queried indices.
    This is still 100x slower compare to fetching in-memory frames.

    >>> Example usage
    >>> video_path = "your_video.mp4"
    >>> frame_indices = [0, 10, 20, 30]
    >>> frames = get_specific_frames_pyav(video_path, frame_indices)

    Parameters:
    - video_path (str): Path to the MP4 file.
    - frame_indices (list of int): List of frame indices to fetch (0-based).
    - percentage (bool) :interpret frame_indices as percentage instead

    Returns:
    - frames (list of np.ndarray): List of the decoded frames as NumPy arrays, in the same order as the input indices.
    """
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        raise ValueError(f"Cannot open video file: {video_path}")

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    if percentage:
        frame_indices = [int(fraction * (total_frames - 1)) for fraction in frame_indices]

    if any(idx < 0 or idx >= total_frames for idx in frame_indices):
        raise IndexError(f"Some frame indices are out of range (0-{total_frames - 1}). Indices: {frame_indices}")

    # sort first can make code a bit faster
    index_to_order = {idx: i for i, idx in enumerate(frame_indices)}
    unique_sorted_indices = sorted(set(frame_indices))

    # Decode frames
    decoded_frames = {}
    for frame_index in unique_sorted_indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        ret, frame = cap.read()
        if not ret:
            raise ValueError(f"Failed to fetch frame {frame_index} from video.")
        decoded_frames[frame_index] = frame

    cap.release()
    frames = [decoded_frames[idx] for idx in frame_indices]
    return np.array(frames)[::-1]


def make_grid(array, ncol=5, padding=0, pad_value=120):
    """numpy version of the make_grid function in torch. Dimension of array: NHWC"""
    if np.max(array) < 2.0:
        array = array * 255.0
    if len(array.shape) == 3:  # In case there is only one channel
        array = np.expand_dims(array, 3)
    N, H, W, C = array.shape
    if N % ncol > 0:
        res = ncol - N % ncol
        array = np.concatenate([array, np.ones([res, H, W, C])])
        N = array.shape[0]
    nrow = N // ncol
    idx = 0
    grid_img = None
    for i in range(nrow):
        row = np.pad(
            array[idx],
            [[padding if i == 0 else 0, padding], [padding, padding], [0, 0]],
            constant_values=pad_value,
            mode="constant",
        )
        for _ in range(1, ncol):
            idx += 1
            cur_img = np.pad(
                array[idx],
                [[padding if i == 0 else 0, padding], [0, padding], [0, 0]],
                constant_values=pad_value,
                mode="constant",
            )
            row = np.hstack([row, cur_img])
        idx += 1
        if i == 0:
            grid_img = row
        # Handle None case for grid_img
        elif grid_img is not None and row is not None:
            grid_img = np.vstack([grid_img, row])
        elif row is not None:
            grid_img = row
    # Handle None case before calling astype
    if grid_img is None:
        return np.zeros((1, 1, 3), dtype=np.float32)  # Return a small empty grid
    return grid_img.astype(np.float32)


def save_numpy_as_gif(array, filename, fps=30, scale=1.0):
    """Creates a gif given a stack of images using moviepy
    Notes
    -----
    works with current Github version of moviepy (not the pip version)
    https://github.com/Zulko/moviepy/commit/d4c9c37bc88261d8ed8b5d9b7c317d13b2cdf62e
    Usage
    -----
    >>> X = randn(100, 64, 64)
    >>> gif("test.gif", X)
    Parameters
    ----------
    filename : string
        The filename of the gif to write to
    array : array_like
        A numpy array that contains a sequence of images
    fps : int
        frames per second (default: 10)
    scale : float
        how much to rescale each image by (default: 1.0)
    """
    from moviepy import ImageSequenceClip

    if np.max(array) <= 2.0:
        array *= 255.0
    # ensure that the file has the .gif extension
    fname, _ = os.path.splitext(filename)
    filename = fname + ".gif"

    # copy into the color dimension if the images are black and white
    if array.ndim == 3:
        array = array[..., np.newaxis] * np.ones(3)

    # make the moviepy clip
    clip = ImageSequenceClip(list(array), fps=fps).resize(scale)
    clip.write_gif(filename, fps=fps)
    return clip


def save_numpy_as_video(array, filename, fps=30, extension="mp4"):
    """Creates a gif given a stack of images using moviepy
    Notes
    -----
    works with current Github version of moviepy (not the pip version)
    https://github.com/Zulko/moviepy/commit/d4c9c37bc88261d8ed8b5d9b7c317d13b2cdf62e
    Usage
    """
    from moviepy import ImageSequenceClip

    if np.max(array) <= 2.0:
        array *= 255.0
    array = array.astype(np.uint8)
    # ensure that the file has the .mp4 extension
    fname, _ = os.path.splitext(filename)
    filename = fname + f".{extension}"

    # copy into the color dimension if the images are black and white
    if array.ndim == 3:
        array = array[..., np.newaxis] * np.ones(3)

    # copy into the color dimension if the images are black and white
    if array.ndim == 3:
        array = array[..., np.newaxis] * np.ones(3)

    # make the moviepy clip without interpalation
    clip = ImageSequenceClip(list(array), fps=fps)
    clip.write_videofile(filename, fps=fps, logger=None)
    return clip


def video_pad_time(videos):
    nframe = np.max([video.shape[0] for video in videos])
    padded = []
    for video in videos:
        npad = nframe - len(video)
        padded_frame = video[[-1], :, :, :].copy()
        video = np.vstack([video, np.tile(padded_frame, [npad, 1, 1, 1])])
        padded.append(video)
    return np.array(padded)


def make_grid_fast_video(videos, ncol=5, pad_value=120):
    """Efficiently creates a grid from multiple videos without looping over frames.
    Input: videos.shape = (N, T, H, W, C) where:
    - N = number of videos
    - T = number of frames
    - H, W, C = frame dimensions
    """
    N, T, H, W, C = videos.shape
    nrow = int(np.ceil(N / ncol))

    # Padding if the number of videos isn't a perfect multiple of ncol
    pad_count = nrow * ncol - N
    if pad_count > 0:
        pad_videos = np.full((pad_count, T, H, W, C), pad_value, dtype=videos.dtype)
        videos = np.concatenate([videos, pad_videos], axis=0)  # Shape: (new_N, T, H, W, C)

    # Reshape to grid layout and bring T to the front
    videos = videos.reshape(nrow, ncol, T, H, W, C).transpose(2, 0, 1, 3, 4, 5)

    grid_frames = videos.transpose(0, 1, 3, 2, 4, 5).reshape(T, nrow * H, ncol * W, C)
    return grid_frames


def make_grid_video_from_numpy(
    video_array: np.ndarray, ncol: int, output_name: str = "./output.mp4", speedup: int = 1, **kwargs: Any
) -> None:
    videos = []
    for video in video_array:
        if speedup != 1:
            video = video[::speedup]
        videos.append(video)
    videos = video_pad_time(videos)  # N x T x H x W x 3
    grid_frames = make_grid_fast_video(videos, ncol=ncol)
    save_numpy_as_video(np.array(grid_frames), output_name, **kwargs)


def make_grid_gif_from_numpy(
    video_array: np.ndarray, ncol: int, output_name: str = "./output.gif", speedup: int = 1, fps: int = 10
) -> None:
    videos = []
    for video in video_array:
        if speedup != 1:
            video = video[::speedup]
        videos.append(video)
    videos = video_pad_time(videos)  # N x T x H x W x 3
    grid_frames = make_grid_fast_video(videos, ncol=ncol)
    save_numpy_as_gif(grid_frames, output_name, fps=fps)


def make_grid_video(
    video_list: list,
    ncol: int,
    output_name: str = "./output.mp4",
    speedup: int = 1,
    video_width: int = 1080,
    **kwargs: Any,
) -> None:
    from moviepy import VideoFileClip

    size0_per_video = video_width // ncol

    videos = []
    for video_path in video_list:
        myclip = VideoFileClip(video_path)
        if myclip.size[0] > size0_per_video:
            myclip = myclip.resized(height=size0_per_video)
        if speedup != 1:
            myclip = myclip.speedx(speedup)
        frames = []
        for frame in myclip.iter_frames():
            frames.append(frame)
        videos.append(np.array(frames))
    videos = video_pad_time(videos)  # N x T x H x W x 3
    grid_frames = make_grid_fast_video(videos, ncol=ncol)
    save_numpy_as_video(grid_frames, output_name, **kwargs)


def get_video_fps_ffmpeg(video_path):
    """Get video FPS using FFmpeg."""
    cmd = [
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=r_frame_rate",
        "-of",
        "default=noprint_wrappers=1:nokey=1",
        video_path,
    ]

    output = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)
    fps_str = output.stdout.strip()

    # Convert fractional FPS (e.g., "30000/1001" ? 29.97)
    if "/" in fps_str:
        num, den = map(int, fps_str.split("/"))
        return num / den
    return float(fps_str)


def get_video_shape(video_path: str) -> List[int]:
    cmd = [
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=width,height",
        "-of",
        "json",
        video_path,
    ]
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, text=True)
    width = int(json.loads(result.stdout)["streams"][0]["width"])
    height = int(json.loads(result.stdout)["streams"][0]["height"])
    return [width, height]


def overlay_wrist_videos_on_top_video(
    top_video_path: str,
    left_wrist_video_path: str,
    right_wrist_video_path: str,
    output_path: str,
    nvenc: bool = False,
    gpu_id: int = 0,
    wrist_video_plot_ratio: int = 4,
):
    """
    generate a low quality video overlay with the top and the wrist cameras.
    the output video has max dimension of 640px.

    -----------------------------------------------------
    left_camera_view                    right_camera_view


                    top_camera_view

    -----------------------------------------------------

    automatically detect if the video is stereo pair and use left image if so.
    """
    main_width, main_height = get_video_shape(top_video_path)
    aspect_ratio = main_width / main_height
    split = 1
    if aspect_ratio > 3:
        split = 2
        main_width = main_width // 2
    overlay_width = int(main_width / wrist_video_plot_ratio)
    env = os.environ.copy()
    env["CUDA_VISIBLE_DEVICES"] = str(gpu_id)
    cmd_task = [
        "ffmpeg",
        "-i",
        top_video_path,
        "-i",
        left_wrist_video_path,
        "-i",
        right_wrist_video_path,
        "-filter_complex",
        f"\
        [0:v]crop=iw/{split}:ih:0:0[bg]; \
        [1:v]scale={overlay_width}:-1[vid_left]; \
        [2:v]scale={overlay_width}:-1[vid_right]; \
        [vid_left]drawtext=text='Left':x=5:y=5:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.5[vid_left_labeled]; \
        [vid_right]drawtext=text='Right':x=5:y=5:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.5[vid_right_labeled]; \
        [bg][vid_left_labeled]overlay=10:10[tmp1]; \
        [tmp1][vid_right_labeled]overlay=W-w-10:10[overlayed]; \
        [overlayed]scale='if(gt(iw,ih),640,-2)':'if(gt(iw,ih),-2,640)':force_divisible_by=2[outv]",
    ]

    if nvenc:
        enc_option = [
            "-map",
            "[outv]",
            "-map",
            "0:a?",
            "-c:v",
            "h264_nvenc",
            "-rc",
            "vbr",
            "-cq",
            "40",  # low res
            "-b:v",
            "0",
            "-preset",
            "p4",
            "-c:a",
            "copy",
            "-shortest",
        ]
    else:
        enc_option = [
            "-map",
            "[outv]",
            "-map",
            "0:a?",
            "-c:v",
            "libx264",
            "-crf",
            "32",
            "-preset",
            "veryslow",
            "-c:a",
            "copy",
            "-shortest",
        ]

    cmd = cmd_task + enc_option + [output_path]

    subprocess.run(cmd, check=True, env=env)


def resize_and_pad_video(
    input_path: str,
    output_path: str,
    target_size: int = 224,
    target_aspect_ratio: tuple[int, int] | None = None,
    encoder: str = "h264_nvenc",
    crf: Optional[int] = None,
    bitrate: Optional[str] = None,
    overwrite: bool = True,
    fps: int | float | str | None = None,
    frame_stride: int | None = None,
    keep_left_half: bool = False,
    crop_to_square: bool = False,
    cpu_encoder: bool = False,
) -> bool:
    """
    Resize, pad, and optionally subsample a video using ffmpeg.

    Args:
        input_path: Path to input video file
        output_path: Path for output video file
        target_size: Target width/height for square output (default: 224)
        target_aspect_ratio: Optional tuple (width, height) for non-square output
        encoder: Video encoder to use (default: "h264_nvenc", fallback: "libx264")
        crf: Constant Rate Factor for quality (0-51, lower is better quality)
        bitrate: Target bitrate (e.g., "2M", "1500k")
        overwrite: Whether to overwrite existing output file
        fps: Target frame rate (e.g., 15, 30, "30000/1001"). If None, keeps original fps
        frame_stride: Take every Nth frame (e.g., stride=2 takes every other frame).
                     Alternative to fps for precise frame subsampling

    Returns:
        bool: True if successful, False otherwise

    Raises:
        FileNotFoundError: If input file doesn't exist
        ValueError: If parameters are invalid

    Note:
        - If both fps and frame_stride are specified, frame_stride takes precedence
        - frame_stride=2 means take every 2nd frame (halves frame rate)
        - frame_stride=3 means take every 3rd frame (1/3 frame rate)
    """

    # Validate inputs
    input_path_obj = Path(input_path)
    output_path_obj = Path(output_path)

    if not input_path_obj.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")

    if target_size <= 0:
        raise ValueError("target_size must be positive")

    if frame_stride is not None and frame_stride <= 0:
        raise ValueError("frame_stride must be positive")

    # Create output directory if it doesn't exist
    output_path_obj.parent.mkdir(parents=True, exist_ok=True)

    # Handle file overwrite
    if output_path_obj.exists() and not overwrite:
        raise FileExistsError(f"Output file exists and overwrite=False: {output_path}")

    # Determine target dimensions
    if target_aspect_ratio:
        target_width, target_height = target_aspect_ratio
    else:
        target_width = target_height = target_size

    # Build ffmpeg command
    cmd = ["ffmpeg"]

    # Input
    cmd.extend(["-i", str(input_path)])

    # Build video filter chain
    filters = []

    # Temporal subsampling filter
    if frame_stride is not None:
        # Use select filter for precise frame selection
        filters.append(f"select='not(mod(n,{frame_stride}))'")
    elif fps is not None:
        # Use fps filter for frame rate conversion
        filters.append(f"fps={fps}")

    if keep_left_half:
        filters.append("crop=iw/2:ih:0:0")

    if crop_to_square:
        filters.append("crop='min(iw\\,ih)':'min(iw\\,ih)':'(iw-min(iw\\,ih))/2':'(ih-min(iw\\,ih))/2'")
        filters.append(f"scale={target_size}:{target_size}")

    else:
        # Resize preserving aspect ratio, then pad to target dimensions
        if target_aspect_ratio:
            target_width, target_height = target_aspect_ratio
            scale_filter = f"scale='if(gt(a,{target_width}/{target_height}),{target_width},-1)':'if(gt(a,{target_width}/{target_height}),-1,{target_height})'"
        else:
            scale_filter = f"scale='if(gt(a,1),{target_size},-1)':'if(gt(a,1),-1,{target_size})'"
        filters.append(scale_filter)
        filters.append(f"pad={target_width}:{target_height}:(ow-iw)/2:(oh-ih)/2")

    # Combine all filters
    vf = ",".join(filters)
    cmd.extend(["-vf", vf])

    # Video encoding options
    if cpu_encoder:
        cmd.extend(["-c:v", "libx264"])
    else:
        cmd.extend(["-c:v", encoder])

    # Quality/bitrate settings
    if crf is not None:
        cmd.extend(["-crf", str(crf)])
    elif bitrate is not None:
        cmd.extend(["-b:v", bitrate])

    # If using frame selection, need to handle timestamps
    if frame_stride is not None:
        cmd.extend(["-vsync", "vfr"])  # Variable frame rate to handle selected frames

    # Overwrite output file
    if overwrite:
        cmd.append("-y")

    # Output
    cmd.append(str(output_path))
    try:
        # Run ffmpeg command on GPU
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return True

    except subprocess.CalledProcessError as e:
        # If NVENC fails, try with CPU encoder as fallback
        if encoder == "h264_nvenc" and "h264_nvenc" in str(e.stderr):
            print("NVENC encoder failed, falling back to libx264...")
            return resize_and_pad_video(
                input_path=str(input_path),
                output_path=str(output_path),
                target_size=target_size,
                target_aspect_ratio=target_aspect_ratio,
                encoder="libx264",
                crf=crf,
                bitrate=bitrate,
                overwrite=overwrite,
                fps=fps,
                frame_stride=frame_stride,
                keep_left_half=keep_left_half,
                crop_to_square=crop_to_square,
            )

        print(f"FFmpeg error: {e.stderr}")
        return False

    except Exception as e:
        print(f"Unexpected error: {e!s}")
        return False


def get_video_resolution(path: str) -> tuple[int, int]:
    cmd = [
        "ffprobe",
        "-v",
        "quiet",
        "-print_format",
        "json",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=width,height",
        path,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, check=True)
    info = json.loads(result.stdout)
    stream = info["streams"][0]
    return stream["width"], stream["height"]