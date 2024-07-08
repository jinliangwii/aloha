import time
import numpy as np
import collections
import matplotlib.pyplot as plt
import dm_env

from telemoma.human_interface.teleop_policy import TeleopPolicy
from importlib.machinery import SourceFileLoader
from telemoma.utils.general_utils import AttrDict
from telemoma.utils.transformations import euler_to_quat, quat_to_euler, add_angles, quat_diff

# from s1_core import S1_Interface_Telemoma
from tracikpy import TracIKSolver
from tf import transformations as T
import math

from constants import DT, START_ARM_POSE, MASTER_GRIPPER_JOINT_NORMALIZE_FN, PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN
from constants import PUPPET_GRIPPER_POSITION_NORMALIZE_FN, PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN
from constants import PUPPET_GRIPPER_JOINT_OPEN, PUPPET_GRIPPER_JOINT_CLOSE
from robot_utils import Recorder, ImageRecorder
from robot_utils import setup_master_bot, setup_puppet_bot, move_arms, move_grippers
from interbotix import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand

import os
disableImageCollecting = "true" #os.getenv("Aloha_Disable_Image_Collection")

class RealEnv:
    """
    Environment for real robot bi-manual manipulation
    Action space:      [left_arm_qpos (6),             # absolute joint position
                        left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)
                        right_arm_qpos (6),            # absolute joint position
                        right_gripper_positions (1),]  # normalized gripper position (0: close, 1: open)

    Observation space: {"qpos": Concat[ left_arm_qpos (6),          # absolute joint position
                                        left_gripper_position (1),  # normalized gripper position (0: close, 1: open)
                                        right_arm_qpos (6),         # absolute joint position
                                        right_gripper_qpos (1)]     # normalized gripper position (0: close, 1: open)
                        "qvel": Concat[ left_arm_qvel (6),         # absolute joint velocity (rad)
                                        left_gripper_velocity (1),  # normalized gripper velocity (pos: opening, neg: closing)
                                        right_arm_qvel (6),         # absolute joint velocity (rad)
                                        right_gripper_qvel (1)]     # normalized gripper velocity (pos: opening, neg: closing)
                        "images": {"cam_high": (480x640x3),        # h, w, c, dtype='uint8'
                                   "cam_low": (480x640x3),         # h, w, c, dtype='uint8'
                                   "cam_left_wrist": (480x640x3),  # h, w, c, dtype='uint8'
                                   "cam_right_wrist": (480x640x3)} # h, w, c, dtype='uint8'
    """

    def __init__(self, init_node, setup_robots=True):

        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.ik_solver = TracIKSolver(dir_path+"/urdf/S1.urdf", "base_link", "Link_EE", timeout=0.025, epsilon=5e-6, solve_type="Distance")

        print("Init bot")
        self.puppet_bot_left = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper",
                                                        robot_name=f'puppet_left', init_node=init_node)

        self.recorder = Recorder('left', init_node=False)

        print( "disableImageCollecting: ", disableImageCollecting )

        if disableImageCollecting != "true":
            print( "init image recorder" )
            self.image_recorder = ImageRecorder(init_node=False)

    def get_qpos(self):
        return self.recorder.qpos

    def get_qvel(self):
        return self.recorder.qvel

    def get_effort(self):
        return self.recorder.effort

    def get_images(self):
        return self.image_recorder.get_images()

    # def set_gripper_pose(self, left_gripper_desired_pos_normalized, right_gripper_desired_pos_normalized):
    #     left_gripper_desired_joint = PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN(left_gripper_desired_pos_normalized)
    #     self.gripper_command.cmd = left_gripper_desired_joint
    #     self.puppet_bot_left.gripper.core.pub_single.publish(self.gripper_command)

    #     right_gripper_desired_joint = PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN(right_gripper_desired_pos_normalized)
    #     self.gripper_command.cmd = right_gripper_desired_joint
    #     self.puppet_bot_right.gripper.core.pub_single.publish(self.gripper_command)

    def _reset_joints(self):
        reset_position = START_ARM_POSE[:6]
        # move_arms([self.puppet_bot_left, self.puppet_bot_right], [reset_position, reset_position], move_time=1)

    def _reset_gripper(self):
        """Set to position mode and do position resets: first open then close. Then change back to PWM mode"""
        # move_grippers([self.puppet_bot_left, self.puppet_bot_right], [PUPPET_GRIPPER_JOINT_OPEN] * 2, move_time=0.5)
        # move_grippers([self.puppet_bot_left, self.puppet_bot_right], [PUPPET_GRIPPER_JOINT_CLOSE] * 2, move_time=1)

    def get_observation(self):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        obs['qvel'] = self.get_qvel()
        obs['effort'] = self.get_effort()
        # if disableImageCollecting != "true" :
            # obs['images'] = self.get_images()
        return obs

    def get_reward(self):
        return 0

    def reset(self, fake=False):
        
        origin_qpos = [1,0,0,0,0,0,0,0]
        self.puppet_bot_left.arm.set_joint_positions(origin_qpos, blocking=False)
        self.wait_for_joint_positions(origin_qpos, timeout=30)

        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())

    def remap_value(self, x):
        if x > 0.5:
            return 0.01
        elif x < -0.5:
            return -0.01
        else:
            return 0.0

    def process_action(self, action):
        # convert deltas to absolute positions
        xyz_delta, euler_delta, gripper = action[:3], action[3:6], action[6]
        # xyz_delta = [0, 0, -0.6]
        xyz_delta = [self.remap_value(x) for x in xyz_delta]
        xyz_delta = [0, 0, xyz_delta[2]]
        euler_delta = [0, 0, 0.0]
        # euler_delta = [x * 0.1 for x in euler_delta]
        # print("xyz_delta: ", xyz_delta)
        # print("euler_delta: ", euler_delta)
        
        cur_pose = self.eef_pose
        cur_xyz = cur_pose[:3]
        cur_quat = cur_pose[3:]
        cur_euler = quat_to_euler(cur_quat)
        # print("cur_xyz: ", cur_xyz)
        # print("cur_euler: ", cur_euler)

        target_xyz = cur_xyz + xyz_delta
        target_euler = add_angles(euler_delta, cur_euler)
        target_quat = euler_to_quat(target_euler)

        # print("target_xyz: ", target_xyz)
        # print("target_euler: ", target_euler)
        # print("target_euler2: ", quat_to_euler(target_quat))

        return target_xyz, target_quat, gripper

    def format_angles(self, angles):
        return [f"{angle:.2f}" for angle in angles]

    def wait_for_joint_positions(self, target_angles, tolerance=0.01, timeout=DT, check_interval=0.1):
        
        start_time = time.time()
        timed_out = False
        
        while True:
            current_angles = self.get_qpos()
            if all(abs(current - target) < tolerance for current, target in zip(current_angles, target_angles)):
                total_time = time.time() - start_time
                if timed_out:
                    print(f"Reached target positions after timeout. Total time: {total_time:.2f} seconds.")
                    print("Timeout: Target qpos:  ", self.format_angles(target_angles))
                    print("Timeout: Current qpos: ", self.format_angles(self.get_qpos()))
                    # exit(1)
                    return False
                else:
                    print(f"Reached target positions in {total_time:.2f} seconds.")
                    break
            
            if not timed_out and time.time() - start_time >= timeout:
                timed_out = True
                print(f"Timeout reached after {timeout} seconds without reaching the target positions.")
                print("Timeout: qpos while timeout: ", self.format_angles(self.get_qpos()))
                
            
            time.sleep(check_interval)

    def step(self, action):
        # print( "relative pose:", action)

        pos, quat, gripper_act = self.process_action(action['right'])
        print( "target pose:", pos, quat, gripper_act)

        # trac-ik
        ee_matrix = T.quaternion_matrix(quat)
        ee_matrix = np.dot(T.translation_matrix(pos), ee_matrix)
        # print(ee_matrix)
        
        ik_solution = self.ik_solver.ik(ee_matrix, qinit=np.zeros(self.ik_solver.number_of_joints))
        print("ik_solution", type(ik_solution), ik_solution)
        
        if ik_solution is None:
            print('No IK solution for ', pos, quat, self.eef_pose)
            ik_solution =  [angle for angle in self.get_qpos()[1:]]
        else:
            ik_solution = ik_solution.tolist()

        # if abs(gripper_act - self.gripper_state) > 0.2:
            # print (gripper_act, self.gripper_state)

        # cur_pos = self.get_qpos()
        # spaceMouseX = action['right'][0] # forward(1) and backwards(-1)
        # spaceMouseY = action['right'][1]  # Left(1) and right(-1)
        spaceMouseZ = action['right'][2]  # up(1) and down(-1)

        angles_degrees = [gripper_act] + [int(math.degrees(angle)) for angle in reversed(ik_solution)]
        # print( "spaceMouseZ: ", spaceMouseZ, " target qpos(degree): ", angles_degrees)
        # print( "target qpos(degree): ", angles_degrees)

        self.puppet_bot_left.arm.set_joint_positions(angles_degrees, blocking=False)
        self.wait_for_joint_positions(angles_degrees)
        
        time.sleep(DT)
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())

    @property
    def eef_pose(self):
        # I probably need to reverse this
        posInDegree = self.get_qpos()
        # posInDegree = [1.0, 0, 44, 0, -58, 0, 13, 0]
        # posInDegree = [1.0, 0, 0, 0, 0, 0, 0, 0]
        robotPosInRadian = list([math.radians(angle) for angle in posInDegree[1:]])[::-1]
        # print( "real qpos", robotPosInRadian )
        # print ( "ee joint positions:", robotPosInRadian )

        # forward kinametics
        eef_matrix = self.ik_solver.fk(robotPosInRadian)

        q = T.quaternion_from_matrix(eef_matrix)
        t = T.translation_from_matrix(eef_matrix)

        return np.concatenate((t, q))

    @property
    def gripper_state(self):
        robotPos = self.get_qpos()
        p = 0
        if robotPos != None:
            p = robotPos[0]

        return p

def make_real_env(init_node, setup_robots=True):
    env = RealEnv(init_node, setup_robots)
    return env
    
def test_real_teleop():
    """
    Test bimanual teleoperation and show image observations onscreen.
    It first reads joint poses from both master arms.
    Then use it as actions to step the environment.
    The environment returns full observations including images.

    An alternative approach is to have separate scripts for teleoperation and observation recording.
    This script will result in higher fidelity (obs, action) pairs
    """

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--teleop_config', type=str, help='Path to the teleop config to use.')
    args = parser.parse_args()

    onscreen_render = True
    render_cam = 'cam_wrist'

    teleop_config = SourceFileLoader('conf', args.teleop_config).load_module().teleop_config
    teleop = TeleopPolicy(teleop_config)
    teleop.start()

    # s1Core = S1_Interface_Telemoma()

    # setup the environment
    env = make_real_env(init_node=True)
    # print("eef pose, gripper state: ", env.eef_pose, env.gripper_state)

    ts = env.reset(fake=True)
    episode = [ts]
    # setup visualization
    # if onscreen_render:
        # ax = plt.subplot()
        # plt_img = ax.imshow(ts.observation['images'][render_cam])
        # plt.ion()

    telemomaEmptyObs = AttrDict({
        'left': np.array([0, 0, 0, 0, 0, 0, 1]),
        'right': np.array([0, 0, 0, 0, 0, 0, 1]),
        'base': np.array([0, 0, 0])
    })

    for t in range(int(1 / DT * 10) ):
        
        telemomaAction = teleop.get_action(telemomaEmptyObs) # Get action from space mouse
        # print( telemomaAction )

        ts = env.step(telemomaAction)
        episode.append(ts)

        if onscreen_render:
            continue
            # plt.imsave(f'frame_{t}.png', ts.observation['images'][render_cam])
            # plt_img.set_data(ts.observation['images'][render_cam])
            # plt.pause(DT)
        else:
            time.sleep(DT)

    # teleop.stop()


if __name__ == '__main__':
    test_real_teleop()

