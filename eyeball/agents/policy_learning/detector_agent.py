import threading
import time
from copy import deepcopy
from typing import Any, Dict, Optional

import numpy as np
import viser
import viser.extras
import viser.transforms as vtf
from dm_env.specs import Array

from eyeball.agents.agent import PolicyAgent
from eyeball.sensors.cameras.camera_utils import obs_get_rgb, resize_with_center_crop
from eyeball.utils.portal_utils import remote
import torch

class DetectorAgent(PolicyAgent):
    def __init__(self):
        self.viser_server = viser.ViserServer()
        self.obs = None
        self.real_vis_thread = threading.Thread(target=self._update_visualization)
        self.real_vis_thread.start()
        self._setup_visualization()
        self.det_model = torch.hub.load("ultralytics/yolov5", "yolov5s")  # Default: yolov5s


    def _setup_visualization(self):
        self.eye_frame_real = self.viser_server.scene.add_frame("/eye_frame_real", show_axes=True, wxyz=vtf.SO3.from_rpy_radians(-np.pi/2, 0.0, 0.0).wxyz)
        self.eye_frame_handle = self.viser_server.scene.add_transform_controls("eye_target_frame", disable_axes=True, disable_sliders=True, wxyz=vtf.SO3.from_rpy_radians(-np.pi/2, 0.0, 0.0).wxyz)

        self.viser_cam_img_handles = {}
        self.viser_frustum_handles = {}

    def _update_visualization(self):
        while self.obs is None:
            time.sleep(0.025)
        while True:
        #     if self.bimanual:
        #         self.urdf_vis_right_real.update_cfg(np.flip(self.obs["right"]["joint_pos"]))
        #     self.urdf_vis_left_real.update_cfg(np.flip(self.obs["left"]["joint_pos"]))

        #     # Extract RGB images from observation (if any)
            rgb_images = obs_get_rgb(self.obs)
            if rgb_images:
                for key in rgb_images.keys():
                    if key not in self.viser_cam_img_handles.keys():
                        self.viser_cam_img_handles[key] = self.viser_server.gui.add_image(rgb_images[key], label=key)
                    
                    if key not in self.viser_frustum_handles.keys():
                        self.viser_frustum_handles[key] = self.viser_server.scene.add_camera_frustum(
                            name = "/eye_frame_real/eye_img_frustum", 
                            fov=0.9, 
                            aspect=1.0,
                            scale=0.75,
                        )
                    results = self.det_model(rgb_images[key])

                    """
                    ['ims', 'pred', 'names', 'files', 'times', 'xyxy', 'xywh', 'xyxyn', 'xywhn', 'n', 't', 's', '__module__', '__doc__', '__init__', '_run', 'show', 'save', 'crop', 'render', 'pandas', 'tolist', 'print', '__len__', '__str__', '__repr__', '__dict__', '__weakref__', '__new__', '__hash__', '__getattribute__', '__setattr__', '__delattr__', '__lt__', '__le__', '__eq__', '__ne__', '__gt__', '__ge__', '__reduce_ex__', '__reduce__', '__getstate__', '__subclasshook__', '__init_subclass__', '__format__', '__sizeof__', '__dir__', '__class__']
                    """

                    # results.print()
                    # print(results.pandas())
                    # for result in results:
                    boxes = results.xyxy  # Boxes object for bounding box outputs
                    names = results.names
                    pred = results.pred
                    # print(results.__dir__())
                    # print(boxes)
                    # print(names)
                    # print(pred)
                    render = results.render()
                    # print(render)
                    # results.show()  # display to screen
                    # results.save("result.jpg")  # save to disk

                    # resize viser images to 224x224
                    # print(render[0].shape)
                    # print(rgb_images[key].shape)
                    # print(rgb_images[key])
                    # viser_img = resize_with_center_crop(rgb_images[key], 224, 224)
                    viser_img = resize_with_center_crop(render[0], 224, 224)

                    self.viser_cam_img_handles[key].image = viser_img
                    self.viser_frustum_handles[key].image = viser_img

            time.sleep(0.02)

    def forward_kinematics(self, joint_pos: np.ndarray) -> np.ndarray:
        # https://www.researchgate.net/profile/Denis-Laurendeau/publication/225126576_The_Agile_Stereo_Pair_for_active_vision/links/00b4953a57b54c53b5000000/The-Agile-Stereo-Pair-for-active-vision.pdf?origin=publication_detail&_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InB1YmxpY2F0aW9uIiwicGFnZSI6InB1YmxpY2F0aW9uRG93bmxvYWQiLCJwcmV2aW91c1BhZ2UiOiJwdWJsaWNhdGlvbiJ9fQ
        # Eqns. (2), (3)
        phi_1 = joint_pos[0]
        phi_2 = np.arctan2(np.tan(joint_pos[1]), np.cos(joint_pos[0]))

        return np.array([phi_1, phi_2])

    def inverse_kinematics(self, phi: np.ndarray) -> np.ndarray:
        phi_1, phi_2 = phi

        theta_1 = phi_1

        theta_2 = np.arctan(np.cos(theta_1) * np.tan(phi_2))

        return np.array([theta_1, theta_2])


    def act(self, obs: Dict[str, Any]) -> Any:
        self.obs = obs

        # print(self.obs["eye"]["joint_pos"])

        eye_kin = self.forward_kinematics(self.obs["eye"]["joint_pos"])

        # print("\nobs eye joint pos\n", self.obs["eye"]["joint_pos"])

        # print("ik eye_kin\n", self.inverse_kinematics(eye_kin))

        self.eye_frame_real.wxyz = (vtf.SO3.from_rpy_radians(-np.pi/2, 0.0, 0.0) @ vtf.SO3.from_rpy_radians(eye_kin[1], -eye_kin[0], 0.0)).wxyz

        cmd_rpy = np.array((vtf.SO3.from_rpy_radians(-np.pi/2, 0.0, 0.0).inverse() @ vtf.SO3(wxyz=self.eye_frame_handle.wxyz)).as_rpy_radians())

        cmd_rpy = np.array([-cmd_rpy[1], cmd_rpy[0], cmd_rpy[2]])

        cmd_pos = self.inverse_kinematics(cmd_rpy[0:2])
        
        # print(cmd_pos)

        cmd_pos = np.array([0.0, 0.0])

        action = {
            "eye": {
                "pos": cmd_pos,
            }
        }

        return action

    @remote(serialization_needed=True)
    def action_spec(self) -> Dict[str, Dict[str, Array]]:
        """Define the action specification."""
        action_spec = {
            "eye": {"pos": Array(shape=(2,), dtype=np.float32)},
        }

        return action_spec