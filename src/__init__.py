from gym.envs.registration import register

register(
    id='panda-robot-v0',
    entry_point='gym_panda_robot.envs:PandaPegInHoleEnv',
)
