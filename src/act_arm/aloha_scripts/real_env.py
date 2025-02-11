import time
import numpy as np
import collections
import matplotlib.pyplot as plt
import dm_env
from pyquaternion import Quaternion

from constants import DT, START_ARM_POSE, MASTER_GRIPPER_JOINT_NORMALIZE_FN, PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN
from constants import PUPPET_GRIPPER_POSITION_NORMALIZE_FN, PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN
from constants import PUPPET_GRIPPER_JOINT_OPEN, PUPPET_GRIPPER_JOINT_CLOSE
from robot_utils import Recorder, ImageRecorder
from robot_utils import setup_master_bot, setup_puppet_bot, move_arms, move_grippers
# import pyrealsense2 as rs


import IPython
e = IPython.embed

class RealEnv:
    """
    Environment for real robot bi-manual manipulation
    Action space:      [left_arm_qpos (6),             # absolute joint position unit rad
                        left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)
                        # right_arm_qpos (6),            # absolute joint position unit rad
                        # right_gripper_positions (1),]  # normalized gripper position (0: close, 1: open)

    Observation space: {"qpos": Concat[ left_arm_qpos (6),          # absolute joint position unit rad
                                        left_gripper_position (1),  # normalized gripper position (0: close, 1: open)
                                        # right_arm_qpos (6),         # absolute joint position unit rad
                                        # right_gripper_qpos (1)]     # normalized gripper position (0: close, 1: open)
                        "qvel": Concat[ left_arm_qvel (6),         # absolute joint velocity (rad)
                                        left_gripper_velocity (1),  # normalized gripper velocity (pos: opening, neg: closing)
                                        # right_arm_qvel (6),         # absolute joint velocity (rad)
                                        # right_gripper_qvel (1)]     # normalized gripper velocity (pos: opening, neg: closing)
                        "images": {"cam_high": (480x640x3),        # h, w, c, dtype='uint8'
                                   "cam_low": (480x640x3),         # h, w, c, dtype='uint8'
                                   "cam_left_wrist": (480x640x3),  # h, w, c, dtype='uint8'
                                   "cam_right_wrist": (480x640x3)} # h, w, c, dtype='uint8'
    """

    def __init__(self, init_node, setup_robots=True, setup_base=False):        
        
        # self.setup_t265()

        self.recorder_left = Recorder('left', init_node=True)
        self.recorder_action = Recorder('action', init_node=False)
        self.image_recorder = ImageRecorder(init_node=False)
        # self.recorder_right = Recorder('right', init_node=False)
        # self.gripper_command = JointSingleCommand(name="gripper")
    
    def setup_t265(self):
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        # if only pose stream is enabled, fps is higher (202 vs 30)
        cfg.enable_stream(rs.stream.pose)
        self.pipeline.start(cfg)
    
    def get_qpos(self):
        left_qpos_raw = self.recorder_left.qpos
        left_arm_qpos = left_qpos_raw[:6]
        # mock
        left_gripper_qpos = np.array([0.0])
        # left_gripper_qpos = np.array([left_qpos_raw[6]]) # this is position not joint
        return np.concatenate([left_arm_qpos, left_gripper_qpos])

    def get_qvel(self):
        left_qvel_raw = self.recorder_left.qvel
        left_arm_qvel = left_qvel_raw[:6]
        # mock
        left_gripper_qvel = np.array([0.0])
        # left_gripper_qvel = np.array([left_qvel_raw[6]]) # this is position not joint
        return np.concatenate([left_arm_qvel, left_gripper_qvel])
    
    def get_effort(self):
        left_effort_raw = self.recorder_left.effort
        left_robot_effort = left_effort_raw[:6]
        return np.concatenate([left_robot_effort])

    def get_images(self):
        return self.image_recorder.get_images()

    def get_base_vel_t265(self):
        raise NotImplementedError
        frames = self.pipeline.wait_for_frames()
        pose_frame = frames.get_pose_frame()
        pose = pose_frame.get_pose_data()
        
        q1 = Quaternion(w=pose.rotation.w, x=pose.rotation.x, y=pose.rotation.y, z=pose.rotation.z)
        rotation = -np.array(q1.yaw_pitch_roll)[0]
        rotation_vec = np.array([np.cos(rotation), np.sin(rotation)])
        linear_vel_vec = np.array([pose.velocity.z, pose.velocity.x])
        is_forward = rotation_vec.dot(linear_vel_vec) > 0

        base_linear_vel = np.sqrt(pose.velocity.z ** 2 + pose.velocity.x ** 2) * (1 if is_forward else -1)
        base_angular_vel = pose.angular_velocity.y
        return np.array([base_linear_vel, base_angular_vel])

    def get_observation(self, get_tracer_vel=False):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        obs['qvel'] = self.get_qvel()
        # obs['effort'] = self.get_effort()
        obs['images'] = self.get_images()
        # obs['base_vel_t265'] = self.get_base_vel_t265()
        # obs['base_vel'] = self.get_base_vel()
        if get_tracer_vel:
            obs['tracer_vel'] = self.get_tracer_vel()
        return obs

    def get_reward(self):
        return 0

    def reset(self, fake=False):
        if not fake:
            # Reboot puppet robot gripper motors
            pass
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())

    def step(self, action, base_action=None, get_tracer_vel=False, get_obs=True):
        if get_obs:
            obs = self.get_observation(get_tracer_vel)
        else:
            obs = None
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=obs)

    def get_action(self):
         action = self.recorder_action.arm_command
         return action


def make_real_env(init_node, setup_robots=True, setup_base=False):
    env = RealEnv(init_node, setup_robots, setup_base)
    return env

