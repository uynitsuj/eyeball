import threading
import time
from abc import ABC, abstractmethod
from copy import deepcopy
from typing import Any, Dict, Optional, List, Tuple

import numpy as np
import viser
import viser.extras
import viser.transforms as vtf
from dm_env.specs import Array
from scipy.optimize import linear_sum_assignment

from huggingface_hub import hf_hub_download
from ultralytics import YOLO
from supervision import Detections
import pandas as pd

from PIL import Image

from eyeball.agents.agent import PolicyAgent
from eyeball.sensors.cameras.camera_utils import obs_get_rgb, resize_with_center_crop
from eyeball.utils.portal_utils import remote
import torch
import cv2


class FixationMethod(ABC):
    """Abstract base class for fixation methods."""
    
    def __init__(self):
        self.previous_detections = []
        self.current_target_id = None
    
    @abstractmethod
    def select_fixation_point(self, detections_df, img_size: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        """
        Select a fixation point from detections.
        
        Args:
            detections_df: Pandas DataFrame with detection results
            img_size: (height, width) of the image
            
        Returns:
            (center_x, center_y) pixel coordinates or None if no target
        """
        pass
    
    def reset(self):
        """Reset the fixation method state."""
        self.previous_detections = []
        self.current_target_id = None


class SmallestAreaFixation(FixationMethod):
    """Fixates on the smallest detected object."""
    
    def select_fixation_point(self, detections_df, img_size: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        if len(detections_df) == 0:
            return None
            
        # Calculate area for each bounding box
        detections_df['area'] = (detections_df['xmax'] - detections_df['xmin']) * (detections_df['ymax'] - detections_df['ymin'])
        
        # Find the row with the smallest area
        smallest_bbox = detections_df.loc[detections_df['area'].idxmin()]
        
        # Calculate center coordinates
        center_x = int((smallest_bbox['xmin'] + smallest_bbox['xmax']) / 2)
        center_y = int((smallest_bbox['ymin'] + smallest_bbox['ymax']) / 2)
        
        print(f"Smallest bbox center: ({center_x}, {center_y})")
        print(f"Object: {smallest_bbox['name']}, Confidence: {smallest_bbox['confidence']:.2f}")
        print(f"Area: {smallest_bbox['area']:.1f} pixels")
        
        return center_x, center_y


class SaccadingFixation(FixationMethod):
    """Fixates on objects using Hungarian matching for temporal tracking and saccading behavior."""
    
    def __init__(self, saccade_interval: int = 60, max_distance_threshold: float = 100.0):
        super().__init__()
        self.saccade_interval = saccade_interval  # frames between saccades
        self.max_distance_threshold = max_distance_threshold  # max distance for matching
        self.frame_count = 0
        self.last_saccade_frame = 0
        self.tracked_objects = {}  # id -> {bbox, center, last_seen}
        self.next_object_id = 0
    
    def _calculate_iou(self, box1: Dict, box2: Dict) -> float:
        """Calculate IoU between two bounding boxes."""
        x1 = max(box1['xmin'], box2['xmin'])
        y1 = max(box1['ymin'], box2['ymin'])
        x2 = min(box1['xmax'], box2['xmax'])
        y2 = min(box1['ymax'], box2['ymax'])
        
        if x2 <= x1 or y2 <= y1:
            return 0.0
            
        intersection = (x2 - x1) * (y2 - y1)
        area1 = (box1['xmax'] - box1['xmin']) * (box1['ymax'] - box1['ymin'])
        area2 = (box2['xmax'] - box2['xmin']) * (box2['ymax'] - box2['ymin'])
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def _calculate_center_distance(self, box1: Dict, box2: Dict) -> float:
        """Calculate Euclidean distance between box centers."""
        center1_x = (box1['xmin'] + box1['xmax']) / 2
        center1_y = (box1['ymin'] + box1['ymax']) / 2
        center2_x = (box2['xmin'] + box2['xmax']) / 2
        center2_y = (box2['ymin'] + box2['ymax']) / 2
        
        return np.sqrt((center1_x - center2_x)**2 + (center1_y - center2_y)**2)
    
    def _update_tracking(self, detections_df):
        """Update object tracking using Hungarian matching."""
        current_detections = []
        for idx, detection in detections_df.iterrows():
            current_detections.append({
                'idx': idx,
                'xmin': detection['xmin'],
                'xmax': detection['xmax'],
                'ymin': detection['ymin'],
                'ymax': detection['ymax'],
                'name': detection['name'],
                'confidence': detection['confidence']
            })
        
        if not self.tracked_objects:
            # First frame - assign new IDs to all detections
            for detection in current_detections:
                self.tracked_objects[self.next_object_id] = {
                    'bbox': detection,
                    'center': ((detection['xmin'] + detection['xmax']) / 2, 
                              (detection['ymin'] + detection['ymax']) / 2),
                    'last_seen': self.frame_count
                }
                self.next_object_id += 1
            return
        
        # Create cost matrix for Hungarian matching
        tracked_ids = list(self.tracked_objects.keys())
        cost_matrix = np.full((len(tracked_ids), len(current_detections)), 1000.0)
        
        for i, obj_id in enumerate(tracked_ids):
            tracked_obj = self.tracked_objects[obj_id]
            for j, detection in enumerate(current_detections):
                # Use combination of IoU and center distance
                iou = self._calculate_iou(tracked_obj['bbox'], detection)
                distance = self._calculate_center_distance(tracked_obj['bbox'], detection)
                
                if distance < self.max_distance_threshold:
                    # Cost is inverse of IoU plus normalized distance
                    cost_matrix[i, j] = (1.0 - iou) + (distance / self.max_distance_threshold)
        
        # Apply Hungarian matching
        if len(tracked_ids) > 0 and len(current_detections) > 0:
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            
            # Update matched objects
            matched_tracked = set()
            matched_detections = set()
            
            for i, j in zip(row_ind, col_ind):
                if cost_matrix[i, j] < 0.8:  # Threshold for valid match
                    obj_id = tracked_ids[i]
                    detection = current_detections[j]
                    
                    self.tracked_objects[obj_id] = {
                        'bbox': detection,
                        'center': ((detection['xmin'] + detection['xmax']) / 2,
                                  (detection['ymin'] + detection['ymax']) / 2),
                        'last_seen': self.frame_count
                    }
                    matched_tracked.add(obj_id)
                    matched_detections.add(j)
            
            # Remove unmatched tracked objects (lost)
            for obj_id in tracked_ids:
                if obj_id not in matched_tracked:
                    if self.current_target_id == obj_id:
                        self.current_target_id = None
                    del self.tracked_objects[obj_id]
            
            # Add new objects for unmatched detections
            for j, detection in enumerate(current_detections):
                if j not in matched_detections:
                    self.tracked_objects[self.next_object_id] = {
                        'bbox': detection,
                        'center': ((detection['xmin'] + detection['xmax']) / 2,
                                  (detection['ymin'] + detection['ymax']) / 2),
                        'last_seen': self.frame_count
                    }
                    self.next_object_id += 1
    
    def select_fixation_point(self, detections_df, img_size: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        self.frame_count += 1
        
        if len(detections_df) == 0:
            self.current_target_id = None
            return None
        
        # Update tracking
        self._update_tracking(detections_df)
        
        # Decide if we should saccade to a new target
        should_saccade = (
            self.current_target_id is None or  # No current target
            self.current_target_id not in self.tracked_objects or  # Current target lost
            (self.frame_count - self.last_saccade_frame) >= self.saccade_interval  # Time for saccade
        )

        if should_saccade and len(self.tracked_objects) > 0:
            print("Saccading to a new target")
            # Choose a new target (prefer objects we haven't looked at recently)
            available_targets = [
                (obj_id, obj_data) for obj_id, obj_data in self.tracked_objects.items()
                if obj_id != self.current_target_id
            ]
            
            if not available_targets and self.current_target_id in self.tracked_objects:
                # Only one object, keep current target
                available_targets = [(self.current_target_id, self.tracked_objects[self.current_target_id])]
            
            if available_targets:
                # Select target with highest confidence or random selection
                target_id, target_data = max(available_targets, 
                                           key=lambda x: x[1]['bbox']['confidence'])
                self.current_target_id = target_id
                self.last_saccade_frame = self.frame_count
                
                print(f"Saccading to object {target_id}: {target_data['bbox']['name']} "
                      f"(confidence: {target_data['bbox']['confidence']:.2f})")
        
        # Return fixation point for current target
        if self.current_target_id in self.tracked_objects:
            target = self.tracked_objects[self.current_target_id]
            center_x, center_y = target['center']
            return int(center_x), int(center_y)
        
        return None
    
    def reset(self):
        super().reset()
        self.frame_count = 0
        self.last_saccade_frame = 0
        self.tracked_objects = {}
        self.next_object_id = 0


class DetectorAgent(PolicyAgent):
    def __init__(self, fixation_method: str = "saccading", 
                 kp: float = 0.5, ki: float = 0.1, kd: float = 0.005):
        self.viser_server = viser.ViserServer()
        self.obs = None
        self.real_vis_thread = threading.Thread(target=self._update_visualization)
        self.real_vis_thread.start()
        self._setup_visualization()

        self.draw = False

        
        # self.det_model = torch.hub.load("ultralytics/yolov5", "yolov5s")  # Default: yolov5s

        model_path = hf_hub_download(repo_id="arnabdhar/YOLOv8-Face-Detection", filename="model.pt")


        self.det_model = YOLO(model_path)

        self.det_model_type = type(self.det_model)

        # print(self.det_model_type)

        self.det_model.conf = 0.55  # NMS confidence threshold
        self.det_model.iou = 0.45  # NMS IoU threshold
        self.det_model.agnostic = False  # NMS class-agnostic
        self.det_model.multi_label = False  # NMS multiple labels per box
        self.cmd_pos = np.array([0.0, 0.0])
        
        # PID Controller parameters
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain  
        self.kd = kd  # Derivative gain
        
        # PID state variables
        self.error_integral = np.array([0.0, 0.0])
        self.previous_error = np.array([0.0, 0.0])
        self.previous_time = None
        
        # Control parameters
        self.eye_actuation_extent = 40 * np.pi / 180.0  # 40 degrees in radians
        self.target_center = None  # Current target position in image coordinates
        
        # Initialize fixation method
        self.set_fixation_method(fixation_method)
        
    def set_fixation_method(self, method_name: str):
        """Hot-swap the fixation method."""
        if method_name == "smallest_area":
            self.fixation_method = SmallestAreaFixation()
        elif method_name == "saccading":
            self.fixation_method = SaccadingFixation(saccade_interval=120, max_distance_threshold=100.0)
        else:
            raise ValueError(f"Unknown fixation method: {method_name}")
        
        print(f"Switched to fixation method: {method_name}")
        
    def get_available_fixation_methods(self) -> List[str]:
        """Get list of available fixation methods."""
        return ["smallest_area", "saccading"]
    
    def configure_saccading_fixation(self, saccade_interval: int = 120, max_distance_threshold: float = 100.0):
        """Configure parameters for saccading fixation method."""
        if isinstance(self.fixation_method, SaccadingFixation):
            self.fixation_method.saccade_interval = saccade_interval
            self.fixation_method.max_distance_threshold = max_distance_threshold
            print(f"Updated saccading parameters: interval={saccade_interval}, threshold={max_distance_threshold}")
        else:
            print("Current fixation method is not saccading. Switch to saccading first.")
    
    def reset_fixation_method(self):
        """Reset the current fixation method's state."""
        self.fixation_method.reset()
        print("Fixation method state reset.")
    
    def _detections_to_dataframe(self, detections: Detections) -> pd.DataFrame:
        """
        Convert supervision Detections object to pandas DataFrame compatible with fixation methods.
        
        Args:
            detections: supervision.Detections object from YOLOv8
            
        Returns:
            pandas DataFrame with columns: xmin, ymin, xmax, ymax, confidence, name
        """
        if len(detections) == 0:
            # Return empty DataFrame with correct columns
            return pd.DataFrame(columns=['xmin', 'ymin', 'xmax', 'ymax', 'confidence', 'name'])
        
        # Extract bounding box coordinates
        xyxy = detections.xyxy  # Shape: (N, 4) - [x1, y1, x2, y2]
        
        # Create DataFrame
        df_data = {
            'xmin': xyxy[:, 0],
            'ymin': xyxy[:, 1], 
            'xmax': xyxy[:, 2],
            'ymax': xyxy[:, 3],
            'confidence': detections.confidence if detections.confidence is not None else [1.0] * len(detections),
        }
        
        # Handle class names
        if detections.class_id is not None:
            # Map class IDs to names if available
            if hasattr(self.det_model, 'names') and self.det_model.names:
                class_names = [self.det_model.names.get(int(class_id), f'class_{class_id}') 
                              for class_id in detections.class_id]
            else:
                # Fallback to generic names or use data if available
                if detections.data is not None and 'class_name' in detections.data:
                    class_names = detections.data['class_name']
                else:
                    class_names = [f'class_{class_id}' for class_id in detections.class_id]
            df_data['name'] = class_names
        else:
            # No class information available
            df_data['name'] = ['unknown'] * len(detections)
        
        df = pd.DataFrame(df_data)
        return df


    def _setup_visualization(self):
        self.eye_frame_real = self.viser_server.scene.add_frame("/eye_frame_real", show_axes=True, wxyz=vtf.SO3.from_rpy_radians(-np.pi/2, 0.0, 0.0).wxyz)
        # self.eye_frame_handle = self.viser_server.scene.add_transform_controls("eye_target_frame", disable_axes=True, disable_sliders=True, wxyz=vtf.SO3.from_rpy_radians(-np.pi/2, 0.0, 0.0).wxyz)

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
                            scale=1.5,
                        )
                    img_size = rgb_images[key].shape
                    results = self.det_model(rgb_images[key])

                    # df = results.pandas()

                    # Convert YOLOv8 results to supervision Detections format
                    detections = Detections.from_ultralytics(results[0])
                    
                    # Convert to pandas DataFrame compatible with fixation methods
                    df = self._detections_to_dataframe(detections)
                    
                    # Use the original RGB image as base for visualization
                    render_img = rgb_images[key].copy()  # Make a writable copy
                    
                    # Draw all detected bounding boxes (optional - for debugging)
                    if self.draw:
                        for _, detection in df.iterrows():
                            x1, y1, x2, y2 = int(detection['xmin']), int(detection['ymin']), int(detection['xmax']), int(detection['ymax'])
                            # Draw bounding box in green
                            cv2.rectangle(render_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            # Draw label
                            label = f"{detection['name']}: {detection['confidence']:.2f}"
                            cv2.putText(render_img, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    
                    # Use the fixation method to select a target
                    fixation_point = self.fixation_method.select_fixation_point(df, img_size)
                    
                    if fixation_point is not None:
                        center_x, center_y = fixation_point
                        
                        if self.draw:
                            # Draw a red circle at the fixation point
                            cv2.circle(render_img, (center_x, center_y), 8, (0, 0, 255), -1)
                            # Draw a smaller white circle inside for better visibility
                            cv2.circle(render_img, (center_x, center_y), 4, (255, 255, 255), -1)
                        
                        # Update command position based on fixation point
                        if self.previous_time is None:
                            self.previous_time = time.time()
                            dt = 0.01
                        else:
                            dt = time.time() - self.previous_time
                            self.previous_time = time.time()
                        
                        current_error = np.array([
                            (center_x - img_size[1]/2) / img_size[1],
                            (center_y - img_size[0]/2) / img_size[0],
                        ])

                        derivative_error = (current_error - self.previous_error) / dt
                        print("derivative_error", derivative_error)

                        self.error_integral += current_error * dt
                        self.error_integral = np.clip(self.error_integral, -self.eye_actuation_extent, self.eye_actuation_extent)

                        self.cmd_pos -= np.array([
                            (self.kp * current_error[0] + self.ki * self.error_integral[0] + self.kd * derivative_error[0]),
                            (self.kp * current_error[1] + self.ki * self.error_integral[1] + self.kd * derivative_error[1]),
                        ])

                        print(self.cmd_pos)

                        self.previous_error = current_error.copy()

                        eye_actuation_extent = 40 * np.pi / 180.0
                        # self.cmd_pos -= np.array([
                        #     (center_x - img_size[1]/2) / img_size[1] * 0.5, 
                        #     (center_y - img_size[0]/2) / img_size[0] * 0.5,
                        # ])
                        # self.cmd_pos -= current_error


                        self.cmd_pos = np.clip(self.cmd_pos, -eye_actuation_extent, eye_actuation_extent)
                        # print(40 * np.pi / 180.0)
                        # print(self.cmd_pos)
                    else:
                        print("No fixation target selected")
                    # print(render)
                    # results.show()  # display to screen
                    # results.save("result.jpg")  # save to disk

                    # resize viser images to 224x224
                    # print(render[0].shape)
                    # print(rgb_images[key].shape)
                    # print(rgb_images[key])
                    # viser_img = resize_with_center_crop(rgb_images[key], 224, 224)

                    # print(f"Screen center offset: ({center_x - img_size[1]/2}, {center_y - img_size[0]/2})")

                    viser_img = resize_with_center_crop(render_img, 300, 300)

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

        # cmd_rpy = np.array((vtf.SO3.from_rpy_radians(-np.pi/2, 0.0, 0.0).inverse() @ vtf.SO3(wxyz=self.eye_frame_handle.wxyz)).as_rpy_radians())

        # cmd_rpy = np.array([-cmd_rpy[1], cmd_rpy[0], cmd_rpy[2]])

        # cmd_pos = self.inverse_kinematics(cmd_rpy[0:2])
        
        # print(cmd_pos)
        # self.cmd_pos = 

        # cmd_pos = np.array([0.0, 0.0])

        action = {
            "eye": {
                "pos": self.cmd_pos,
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