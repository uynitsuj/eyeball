import threading
import time
from copy import deepcopy
from typing import Any, Dict, Optional

import numpy as np
import viser
import viser.extras
import viser.transforms as vtf
from dm_env.specs import Array

from eyeball.agents.agent import Agent
from eyeball.sensors.cameras.camera_utils import obs_get_rgb, resize_with_pad
from eyeball.utils.portal_utils import remote

class ViserEyeballAgent(Agent):
    def __init__(self):
        self.viser_server = viser.ViserServer()
        self.obs = None
        self.real_vis_thread = threading.Thread(target=self._update_visualization)
        self.real_vis_thread.start()
        self._setup_visualization()

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
                    # resize viser images to 224x224
                    viser_img = resize_with_pad(rgb_images[key], 224, 224)

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

        action = {
            "eye": {
                "pos": np.array(cmd_pos),
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