from gym.envs.registration import register

register(
    id='Mobilerobot_Pybullet-v0',
    entry_point='gym_pybullet_mobilerobot.envs:MobileRoboGymEnv'
)
