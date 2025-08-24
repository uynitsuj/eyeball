import enum
import logging
import os
import queue
import time
from functools import partial
from typing import Any, Callable, Dict, List, Optional, Tuple, Union, runtime_checkable

import enum
from abc import abstractmethod
from typing import Any, Dict, Protocol, Union, runtime_checkable

import numpy as np
from dm_env.specs import Array

# from i2rt.robots.robot import Robot
# from i2rt.robots.utils import JointMapper

# RPC Method Serialization Requirements.
ROBOT_PROTOCOL_METHODS = {
    "num_dofs": False,
    "get_joint_pos": False,
    "get_joint_state": False,
    "command_joint_pos": False,
    "command_joint_state": False,
    "get_observations": False,
    "joint_pos_spec": True,
    "joint_state_spec": True,
    "get_robot_info": True,
    "get_robot_type": True,
    "command_target_vel": False,
}

import enum
from abc import abstractmethod
from typing import Any, Dict, Protocol, Union, runtime_checkable

import numpy as np
from dm_env.specs import Array

ActionSpec = Union[Array, Dict[str, "ActionSpec"]]
"""Action specification for the agent/robot. It also includes the action space for the gripper."""


@enum.unique
class RobotType(enum.Enum):
    ARM = "arm"
    MOBILE_BASE = "mobile_base"


@runtime_checkable
class Robot(Protocol):
    """A generic Robot protocol."""

    @abstractmethod
    def num_dofs(self) -> int:
        """Get the number of controllable degrees of freedom of the robot.

        Returns:
            int: The number of controllable degrees of freedom of the robot.
        """
        raise NotImplementedError

    def get_joint_pos(self) -> np.ndarray:
        """Get the current joint positions of the robot in radians.

        Returns:
            np.ndarray: The current joint positions of the robot in radians.
        """
        pass

    def get_joint_state(self) -> Dict[str, np.ndarray]:
        """Get the current joint positions and velocities of the robot in radians.

        Returns:
            Dict[str, np.ndarray]: A dictionary containing the current joint positions and velocities of the robot in radians.
        """
        pass

    def command_joint_pos(self, joint_pos: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_pos (np.ndarray): The state to command the leader robot to.
        """
        pass

    def command_target_vel(self, joint_vel: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_vel (np.ndarray): The state to command the leader robot to.
        """
        pass

    def command_joint_state(self, joint_state: Dict[str, np.ndarray]) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (Dict[str, np.ndarray]): The state to command the leader robot to.
        """
        pass

    @abstractmethod
    def get_observations(self) -> Dict[str, np.ndarray]:
        """Get the current observations of the robot.

        This is to extract all the information that is available from the robot,
        such as joint positions, joint velocities, etc. This may also include
        information from additional sensors, such as cameras, force sensors, etc.

        Returns:
            Dict[str, np.ndarray]: A dictionary of observations.
        """
        raise NotImplementedError

    def joint_pos_spec(self) -> ActionSpec:
        """Return the action specification for the robot, which includes the gripper."""
        return Array(
            shape=(self.num_dofs(),),
            dtype=np.float32,
        )

    def joint_state_spec(self) -> ActionSpec:
        """Return the action specification for the robot, which includes the gripper."""
        return dict(
            {
                "pos": Array(
                    shape=(self.num_dofs(),),
                    dtype=np.float32,
                ),
                "vel": Array(
                    shape=(self.num_dofs(),),
                    dtype=np.float32,
                ),
            }
        )

    def get_robot_info(self) -> Dict[str, Any]:
        """Get the robot information, such as kp, kd, joint limits, gripper limits, etc."""
        return {}

    def get_robot_type(self) -> RobotType:
        """Get the robot type."""
        return RobotType.ARM

class JointMapper:
    def __init__(self, index_range_map: Dict[int, Tuple[float, float]], total_dofs: int):
        """_summary_
        This class is used to map the joint positions from the command space to the robot joint space.

        Args:
            index_range_map (Dict[int, Tuple[float, float]]): 0 indexed
            total_dofs (int): num of joints in the robot including the gripper if the girpper is the second robot
        """
        self.empty = len(index_range_map) == 0
        if not self.empty:
            self.joints_one_hot = np.zeros(total_dofs).astype(bool)
            self.joint_limits = []
            for idx, (start, end) in index_range_map.items():
                self.joints_one_hot[idx] = True
                self.joint_limits.append((start, end))
            self.joint_limits = np.array(self.joint_limits)
            self.joint_range = self.joint_limits[:, 1] - self.joint_limits[:, 0]

    def to_robot_joint_pos_space(self, command_joint_pos: np.ndarray) -> np.ndarray:
        if self.empty:
            return command_joint_pos
        command_joint_pos = np.asarray(command_joint_pos, order="C")
        result = command_joint_pos.copy()
        needs_remapping = command_joint_pos[self.joints_one_hot]
        needs_remapping = needs_remapping * self.joint_range + self.joint_limits[:, 0]
        result[self.joints_one_hot] = needs_remapping
        return result

    def to_robot_joint_vel_space(self, command_joint_vel: np.ndarray) -> np.ndarray:
        if self.empty:
            return command_joint_vel
        result = command_joint_vel.copy()
        needs_remapping = command_joint_vel[self.joints_one_hot]
        needs_remapping = needs_remapping * self.joint_range
        result[self.joints_one_hot] = needs_remapping
        return result

    def to_command_joint_vel_space(self, robot_joint_vel: np.ndarray) -> np.ndarray:
        if self.empty:
            return robot_joint_vel
        result = robot_joint_vel.copy()
        needs_remapping = robot_joint_vel[self.joints_one_hot]
        needs_remapping = needs_remapping / self.joint_range
        result[self.joints_one_hot] = needs_remapping
        return result

    def to_command_joint_pos_space(self, robot_joint_pos: np.ndarray) -> np.ndarray:
        if self.empty:
            return robot_joint_pos
        result = robot_joint_pos.copy()
        needs_remapping = robot_joint_pos[self.joints_one_hot]
        needs_remapping = (needs_remapping - self.joint_limits[:, 0]) / self.joint_range
        result[self.joints_one_hot] = needs_remapping
        return result

class PrintRobot(Robot):
    """A robot that prints the commanded joint state."""

    def __init__(self, num_dofs: int, dont_print: bool = False):
        self._num_dofs = num_dofs
        self._joint_state = np.zeros((num_dofs,))
        self._dont_print = dont_print

    def num_dofs(self) -> int:
        return self._num_dofs

    def get_joint_pos(self) -> np.ndarray:
        return self._joint_state

    def command_joint_pos(self, joint_pos: np.ndarray) -> None:
        assert len(joint_pos) == (self._num_dofs), (
            f"Expected joint state of length {self._num_dofs}, got {len(joint_pos)}."
        )
        self._joint_state = joint_pos
        if not self._dont_print:
            print(self._joint_state)

    def get_observations(self) -> Dict[str, np.ndarray]:
        joint_pos = self.get_joint_pos()
        return {
            "joint_pos": joint_pos,
            "joint_vel": joint_pos,
        }


class ConcatenatedRobot(Robot):
    def __init__(self, robots: List[Robot], remapper: Optional[JointMapper] = None):
        self._robots = robots
        self._remapper = remapper
        self.per_robot_index = np.array([i.num_dofs() for i in self._robots]).cumsum()

    def num_dofs(self) -> int:
        return sum(robot.num_dofs() for robot in self._robots)

    def get_joint_pos(self) -> np.ndarray:
        robot_space_joint_pos = np.concatenate([robot.get_joint_pos() for robot in self._robots])
        if self._remapper is not None:
            return self._remapper.to_command_joint_pos_space(robot_space_joint_pos)
        return robot_space_joint_pos

    def command_joint_pos(self, joint_pos: np.ndarray) -> None:
        if self._remapper is not None:
            joint_pos = self._remapper.to_robot_joint_pos_space(joint_pos)
        for robot, pos in zip(self._robots, np.split(joint_pos, self.per_robot_index), strict=False):
            robot.command_joint_pos(pos)

    def command_joint_state(self, joint_state: Dict[str, np.ndarray]) -> None:
        assert self._remapper is None, "Remapper is not supported for command_joint_state"
        for robot, state in zip(self._robots, np.split(joint_state, self.per_robot_index), strict=False):  # type: ignore
            robot.command_joint_state(state)

    def get_observations(self) -> Dict[str, np.ndarray]:
        obs = [robot.get_observations() for robot in self._robots]
        obs_dict = {}
        for o in obs:
            for k, v in o.items():
                if k in obs_dict:
                    obs_dict[k] = np.concatenate([obs_dict[k], v])
                else:
                    obs_dict[k] = v
        return obs_dict