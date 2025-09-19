from array import array
import math
import time, sys, os
from struct import *
import struct
import numpy as np
import serial, json
from concurrent.futures import ThreadPoolExecutor, Future
import threading
from eyeball.robot.lkmotor_driver import LKMotorChain
from eyeball.robot.robot import Robot, ActionSpec
from dm_env.specs import Array
from typing import Dict, Any

class Eyeball(Robot):
    def __init__(
        self, 
        port_name: str = "ttyUSB0"):

        print(f"Initializing Eyeball robot on port {port_name}")
        self.robot = LKMotorChain(port_name)
        self.robot.add_motor(0x01, tolerance=0.1, name="motor1", CW=True, zero_angle=0.0, encoder_bits=15, angle_range="180", joint_limits_deg=(-40.0, 40.0))
        self.robot.add_motor(0x02, tolerance=0.1, name="motor2", CW=True, zero_angle=0.0, encoder_bits=15, angle_range="180", joint_limits_deg=(-40.0, 40.0))
        self.speed = 1000.0

        self.total_dof = len(self.robot.motors)

        self._last_joint_pos = np.zeros(self.total_dof)
        self._last_joint_vel = np.zeros(self.total_dof)


    def goto_zero(self):
        self.robot.goto_zero()

    def goto_abs_multi_loop_angles_speeds(self, angles: list, speeds: list = None):
        return self.robot.goto_abs_multi_loop_angles_speeds(angles, speeds)

        
    def num_dofs(self) -> int:
        """Get the number of controllable degrees of freedom of the robot.

        Returns:
            int: The number of controllable degrees of freedom of the robot.
        """
        return self.total_dof

    def get_joint_pos(self) -> np.ndarray:
        """Get the current joint positions of the robot in radians.

        Returns:
            np.ndarray: The current joint positions of the robot in radians.
        """
        return self._last_joint_pos
    
    def get_joint_vel(self) -> np.ndarray:
        """Get the current joint velocities of the robot in radians per second.

        Returns:
            np.ndarray: The current joint velocities of the robot in radians per second.
        """
        return self._last_joint_vel

    def get_joint_state(self) -> Dict[str, np.ndarray]:
        """Get the current joint positions and velocities of the robot in radians.

        Returns:
            Dict[str, np.ndarray]: A dictionary containing the current joint positions and velocities of the robot in radians.
        """
        return {"joint_pos": self.get_joint_pos(), "joint_vel": self.get_joint_vel()}

    def command_joint_pos(self, joint_pos: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_pos (np.ndarray): The state to command the leader robot to in radians.
        """
        ret = self.robot.goto_abs_multi_loop_angles_speeds(list(joint_pos * 180.0 / np.pi), speeds=[self.speed, self.speed])
        for i, ret_val in enumerate(ret):
            self._last_joint_pos[i] = ret_val["angle_rad"]
            self._last_joint_vel[i] = ret_val["speed_radps"]

    # def command_target_vel(self, joint_vel: np.ndarray) -> None:
    #     """Command the leader robot to a given state.

    #     Args:
    #         joint_vel (np.ndarray): The state to command the leader robot to.
    #     """
    #     pass

    def command_joint_state(self, joint_state: Dict[str, np.ndarray]) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (Dict[str, np.ndarray]): The state to command the leader robot to.
        """
        joint_pos = joint_state["joint_pos"]
        joint_vel = joint_state["joint_vel"]
        ret = self.robot.goto_abs_multi_loop_angles_speeds(list(joint_pos), list(joint_vel))
        for i, ret_val in enumerate(ret):
            self._last_joint_pos[i] = ret_val["angle_rad"]
            self._last_joint_vel[i] = ret_val["speed_radps"]

    def get_observations(self) -> Dict[str, np.ndarray]:
        """Get the current observations of the robot.

        This is to extract all the information that is available from the robot,
        such as joint positions, joint velocities, etc. This may also include
        information from additional sensors, such as cameras, force sensors, etc.

        Returns:
            Dict[str, np.ndarray]: A dictionary of observations.
        """
        return {"joint_pos": self.get_joint_pos()}

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
        return {
            "num_dofs": self.num_dofs(),
            "motor_info": [motor.read_driver_motor_info() for motor in self.robot.motors]}

    # def get_robot_type(self) -> RobotType:
    #     """Get the robot type."""
    #     return RobotType.ARM


# MAIN
if __name__ == '__main__':
    eye = Eyeball()

    eye.goto_zero()

    eye.command_joint_pos(np.array([10.0 * np.pi / 180.0, 20.0 * np.pi / 180.0]))

    # print(eye.joint_pos_spec())
    # print(eye.joint_state_spec())
    # print(eye.get_robot_info())
    # print(eye.get_observations())
    # print(eye.get_joint_pos())
    # print(eye.get_joint_vel())
    # print(eye.get_joint_state())

    ang = np.linspace(-40.0, 40.0, 8)

    counter = 0
    up = True
    # import time
    spd = 1000.0
    while True:
        t0 = time.time()
        eye.command_joint_pos(np.array([ang[counter] * np.pi / 180.0, ang[counter] * np.pi / 180.0]))
        # eye.command_joint_state({"joint_pos": np.array([ang[counter], ang[counter]]), "joint_vel": np.array([spd, spd])})
        print(eye.get_joint_state())
        t1 = time.time()
        print(f"hz: {1 / (t1 - t0)}")

        if counter == len(ang) - 1 and up:
            up = False
        elif counter == 0 and not up:
            up = True

        if up:
            counter += 1
        else:
            counter -= 1

        # time.sleep(1/100)
