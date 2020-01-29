from gym.envs.registration import register

register(
    id='panda-robot-reacher-v0',
    entry_point='gym_panda_robot.envs:PandaReacherEnv',
)

register(
    id='panda-robot-reacher-v1',
    entry_point='gym_panda_robot.envs:PandaReacherEnv1',
)

