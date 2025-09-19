import glob
import logging
import os
import subprocess
import threading
import time

from eyeball.utils.video_utils import save_jpgs_to_mp4_ffmpeg

def async_complie_jpgs_to_mp4(camera_save_dir, delete_jpeg_dir):
    assert os.path.exists(camera_save_dir), f"Camera save directory {camera_save_dir} does not exist"
    save_dir = os.path.basename(camera_save_dir)  # parent episode dir
    camera_imagekey_dirs = glob.glob(os.path.join(camera_save_dir, "*"))

    def save():
        for camera_imagekey_dir in camera_imagekey_dirs:
            camera_imagekey_dir_loc = camera_imagekey_dir
            image_list = glob.glob(os.path.join(camera_imagekey_dir_loc, "*.jpg"))
            image_list.sort(key=lambda x: int(x.split("-")[-1].split(".")[0]))
            print(f"image_list: {image_list[:3]}")
            save_name = os.path.basename(image_list[0]).split("-")[:-1]
            save_name = "-".join(save_name)

            time_stamp_first = int(image_list[0].split("-")[-1].split(".")[0])
            time_stamp_last = int(image_list[-1].split("-")[-1].split(".")[0])

            time_span = time_stamp_last - time_stamp_first
            fps = len(image_list) / (time_span / 1000)

            logging.info(f"Start saving video: {camera_imagekey_dir_loc}, fps: {fps}, time_span: {time_span} ms")
            now = time.time()
            save_jpgs_to_mp4_ffmpeg(camera_imagekey_dir_loc, os.path.join(save_dir, f"{save_name}.mp4"), fps, crf=18)
            logging.info(f"Save video: {camera_imagekey_dir_loc} done, takes {time.time() - now} seconds")
            if delete_jpeg_dir:
                # remove key_save_dir
                logging.info(f"Removing jpeg dir: {camera_imagekey_dir_loc}")
                proc = subprocess.run(["rm", "-rf", camera_imagekey_dir_loc], check=False)

                # if camera_save_dir empty, remove it
                if len(os.listdir(camera_save_dir)) == 0:
                    logging.info(f"Removing camera save dir: {camera_save_dir}")
                    subprocess.run(["rm", "-rf", camera_save_dir], check=False)

    save_thread = threading.Thread(target=save)
    save_thread.start()