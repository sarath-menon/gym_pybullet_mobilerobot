from gym_pybullet_mobilerobot.envs.helper import helper
import numpy as np
import pybullet as p
import time
import gym
from gym import spaces
from gym.utils import seeding

p.connect(p.DIRECT)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)

MAX_STEPS = 1500


class MobileRoboGymEnv(helper,gym.Env):
    """
    Gym wrapper for Mobile Robot environment with 2 targets
    WARNING: to be compatible with kuka scripts, additional keyword arguments are discarded
    :param urdf_root: (str) Path to pybullet urdf files
    :param renders: (bool) Whether to display the GUI or not
    :param is_discrete: (bool) Whether to use discrete or continuous actions
    :param name: (str) name of the folder where recorded data will be stored
    :param max_distance: (float) Max distance between end effector and the button (for negative reward)
    :param shape_reward: (bool) Set to true, reward = -distance_to_goal
    :param use_srl: (bool) Set to true, use srl_models
    :param srl_model_path: (str) Path to the srl model
    :param record_data: (bool) Set to true, record frames with the rewards.
    :param use_ground_truth: (bool) Set to true, the observation will be the ground truth (arm position)
    :param random_target: (bool) Set the target to a random position
    :param state_dim: (int) When learning states
    :param learn_states: (bool)
    :param verbose: (bool) Whether to print some debug info
    :param save_path: (str) location where the saved data should go
    :param env_rank: (int) the number ID of the environment
    :param pipe: (Queue, [Queue]) contains the input and output of the SRL model
    :param fpv: (bool) enable first person view camera
    :param srl_model: (str) The SRL_model used
    """
    def __init__(self):
        helper.__init__(self)
        self.lastLidarTime = time.time()
        self.prev_dist = 0.0
        self.prev_action = np.array([0.,0.])
        obs_high = np.concatenate((np.array([50.] * 10) ,np.array([10.0, 10.0]), np.array([6.]), np.array([4]) ))
        obs_low = np.concatenate((np.array([0.] * 10) ,np.array([0.0, -10.0]), np.array([-6.]), np.array([-4]) ))
        self.action_space = spaces.Box(low=np.array([0.0, -0.5]) ,high=np.array([1.0, 0.5]), dtype=np.float32)
        self.observation_space = spaces.Box(low=obs_low , high=obs_high, dtype=np.float32)
        self.seed()

    def reset(self):
        p.resetSimulation()
        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setTimeStep(1. / 240.)
        p.setGravity(0, 0, -10)

        self.load_walls()
        self.load_robot()
        trans,rot = p.getBasePositionAndOrientation(self.robot_uid)
        self.target = self.load_target(trans)

        dist = np.linalg.norm(np.array([trans[0],trans[1]]) - self.target)
        self.rayFrom,self.rayTo = self.init_laserscanner()

        _,_,yaw = p.getEulerFromQuaternion(rot)
        goal_angle = np.arctan2(self.target[1] - trans[1], self.target[0] - trans[0]) *(180/np.pi)
        heading = goal_angle - yaw

        # for _ in range(50):
        p.stepSimulation()
        obs = self.read_laser_scan()
        # print("Robot reset")
        # print('shape:',obs.shape,dist.shape)
        return np.concatenate((obs, self.prev_action, dist,heading),axis=None)

    def step(self,action):
        done = False
        # print('Before_diff:',action)
        self.prev_action = action
        p.setJointMotorControl2(self.robot_uid,0,p.VELOCITY_CONTROL,targetVelocity=action[0],force=1000)
        p.setJointMotorControl2(self.robot_uid,1,p.VELOCITY_CONTROL,targetVelocity=action[1],force=1000)
        for _ in range(5):
            p.stepSimulation()
        # time.sleep(0.0001)
        # self._env_step_counter += 1
        trans,rot = p.getBasePositionAndOrientation(self.robot_uid)
        _,_,yaw = p.getEulerFromQuaternion(rot)
        goal_angle = np.arctan2(self.target[1] - trans[1], self.target[0] - trans[0])
        heading = np.array([goal_angle - yaw])

        dist = np.linalg.norm(np.array([trans[0],trans[1]]) - self.target)
        # nowLidarTime = time.time()
        obs = self.read_laser_scan()
        dist_rate = dist - self.prev_dist
        # print('prev_dist:',self.prev_dist,'dist:',dist)
        self.prev_dist = dist

        if trans[0]>(self.max_x-0.3) or trans[0]<0.3 or trans[1]>(self.max_x-0.3) or trans[1]<0.3: # If collided with wall
            reward = -550
            print("---------------------------------------")
            print('Collision')
            print("---------------------------------------")
            done = True

        elif dist <= 0.4:   # reached target
            reward = 500
            print('............Goal................')
            done = True

        elif dist_rate > 0:
            reward = 200.*dist_rate

        elif dist_rate <= 0:
            reward = -8.
            # print('reward:',reward)

        else:
            print("Some error")

        return np.concatenate((obs,self.prev_action,dist, heading),axis=None) , reward , done ,{}
