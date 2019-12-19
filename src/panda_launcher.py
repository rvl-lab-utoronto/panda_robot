#!/usr/bin/env python
from copy import copy  # for python2
import numpy as np

from yw.env.env_manager import EnvManager


class FetchPickAndPlaceDemoGenerator:
    """
    Provide an easier way to control the gripper for pick and place task
    """

    def __init__(self, env, system_noise_scale, render):
        self.env = env
        self.system_noise_scale = system_noise_scale
        self.render = render
        self.num_itr = 0

    def reset(self):
        self.num_itr = 0

    def generate_move(self):
        self._reset()
        for i in range(self.num_object):
            goal_dim = 3 * i
            obj_pos_dim = 10 + 15 * i
            obj_rel_pos_dim = 10 + 15 * i + 3
            # move to the above of the object
            self._move_to_object(obj_pos_dim, obj_rel_pos_dim, offset=0.05, gripper_open=True)
            # grab the object
            self._move_to_object(obj_pos_dim, obj_rel_pos_dim, offset=0.0, gripper_open=False)
            # move to the goal
            self._move_to_goal(obj_pos_dim, goal_dim)
            # open the gripper
            self._move_to_object(obj_pos_dim, obj_rel_pos_dim, offset=0.05, gripper_open=True)
            # # move back to initial state
            # self._move_back()
        # stay until the end
        self._stay()

        self.num_itr += 1
        assert self.episode_info[-1]["is_success"]
        return self.episode_obs, self.episode_act, self.episode_rwd, self.episode_info

    def generate_move_auto_place(self):
        self._reset()
        for i in range(self.num_object):
            goal_dim = 3 * i
            obj_pos_dim = 10 + 15 * i
            obj_rel_pos_dim = 10 + 15 * i + 3
            # move to the above of the object
            self._move_to_object(obj_pos_dim, obj_rel_pos_dim, offset=0.05, gripper_open=True)
            # grab the object
            self._move_to_object(obj_pos_dim, obj_rel_pos_dim, offset=0.0, gripper_open=False)
            # move to the goal
            self._move_to_goal(obj_pos_dim, goal_dim)
            # open the gripper
            self._move_to_object(obj_pos_dim, obj_rel_pos_dim, offset=0.05, gripper_open=True)
            # move back to initial state
            self._move_back()
        # stay until the end
        self._stay()

        self.num_itr += 1
        assert self.episode_info[-1]["is_success"]
        return self.episode_obs, self.episode_act, self.episode_rwd, self.episode_info

    def generate_pick_place(self):

        goal_dim = 0
        obj_pos_dim = 10
        obj_rel_pos_dim = 13

        self._reset()
        # move to the above of the object
        self._move_to_object(obj_pos_dim, obj_rel_pos_dim, offset=0.05, gripper_open=True)
        # grab the object
        self._move_to_object(obj_pos_dim, obj_rel_pos_dim, offset=0.0, gripper_open=False)
        # move to the goal
        if self.num_itr % 2 == 0:
            weight = np.array((0.8, 0.2, 0.5)) + np.random.normal(scale=0.05, size=3)
        elif self.num_itr % 2 == 1:
            weight = np.array((0.2, 0.8, 0.5)) + np.random.normal(scale=0.05, size=3)
        else:
            assert False
        self._move_to_interm_goal(obj_pos_dim, goal_dim,weight)
        self._move_to_goal(obj_pos_dim, goal_dim)
        # open the gripper
        self._move_to_object(obj_pos_dim, obj_rel_pos_dim, offset=0.03, gripper_open=True)
        # stay until the end
        self._stay()

        self.num_itr += 1
        assert self.episode_info[-1]["is_success"]
        return self.episode_obs, self.episode_act, self.episode_rwd, self.episode_info

    def generate_peg_in_hole(self):

        goal_dim = 0
        obj_pos_dim = 0

        self._reset()
        # move to the goal
        self._move_to_goal(obj_pos_dim, goal_dim, offset=np.array((0.0,0.0,np.random.uniform(0.15, 0.2))))
        self._move_to_goal(obj_pos_dim, goal_dim)
        # stay until the end
        self._stay()

        self.num_itr += 1
        assert self.episode_info[-1]["is_success"]
        return self.episode_obs, self.episode_act, self.episode_rwd, self.episode_info

    def _reset(self):
        # store return from 1 episode
        self.episode_act = []
        self.episode_obs = []
        self.episode_info = []
        self.episode_rwd = []
        # clear time step
        self.time_step = 0
        # reset env and store initial observation
        obs = self.env.reset()
        self.episode_obs.append(obs)
        # self.num_object = int(obs["desired_goal"].shape[0] / 3)
        self.init_obs = copy(obs)
        self.last_obs = copy(obs)

    def _step(self, action):
        obs, reward, done, info = self.env.step(action)
        self.time_step += 1
        if self.render:
            self.env.render()
        # update stats for the episode
        self.episode_act.append(action)
        self.episode_info.append(info)
        self.episode_obs.append(obs)
        self.episode_rwd.append([reward])
        # update last obs
        self.last_obs = obs

    def _move_to_object(self, obj_pos_dim, obj_rel_pos_dim, offset, gripper_open):
        object_pos = self.last_obs["observation"][obj_pos_dim : obj_pos_dim + 3]
        object_rel_pos = self.last_obs["observation"][obj_rel_pos_dim : obj_rel_pos_dim + 3]
        object_oriented_goal = copy(object_rel_pos)
        object_oriented_goal[2] += offset

        while np.linalg.norm(object_oriented_goal) >= 0.01 and self.time_step <= self.env._max_episode_steps:

            action = [0, 0, 0, 0]
            for i in range(len(object_oriented_goal)):
                action[i] = object_oriented_goal[i] * 10 + self.system_noise_scale * np.random.normal()
            action[-1] = 0.05 if gripper_open else -0.05  # open

            self._step(action)

            object_pos = self.last_obs["observation"][obj_pos_dim : obj_pos_dim + 3]
            object_rel_pos = self.last_obs["observation"][obj_rel_pos_dim : obj_rel_pos_dim + 3]
            object_oriented_goal = copy(object_rel_pos)
            object_oriented_goal[2] += offset

    def _move_to_goal(self, obj_pos_dim, goal_dim, gripper=-1.0, offset=np.array((0.0, 0.0, 0.0))):
        goal = self.last_obs["desired_goal"][goal_dim : goal_dim + 3] + offset
        object_pos = self.last_obs["observation"][obj_pos_dim : obj_pos_dim + 3]

        while np.linalg.norm(goal - object_pos) >= 0.01 and self.time_step <= self.env._max_episode_steps:

            action = [0, 0, 0, 0]
            for i in range(len(goal - object_pos)):
                action[i] = (goal - object_pos)[i] * 10 + self.system_noise_scale * np.random.normal()
            action[-1] = gripper

            self._step(action)

            object_pos = self.last_obs["observation"][obj_pos_dim : obj_pos_dim + 3]

    def _move_to_interm_goal(self, obj_pos_dim, goal_dim, weight):

        goal = self.last_obs["desired_goal"][goal_dim : goal_dim + 3]
        object_pos = self.last_obs["observation"][obj_pos_dim : obj_pos_dim + 3]

        interm_goal = object_pos * weight + goal * (1 - weight)

        while np.linalg.norm(interm_goal - object_pos) >= 0.01 and self.time_step <= self.env._max_episode_steps:

            action = [0, 0, 0, 0]
            for i in range(len(interm_goal - object_pos)):
                action[i] = (interm_goal - object_pos)[i] * 10 + self.system_noise_scale * np.random.normal()
            action[-1] = -0.05

            self._step(action)

            object_pos = self.last_obs["observation"][obj_pos_dim : obj_pos_dim + 3]

    def _move_back(self):
        goal = self.init_obs["observation"][0:3]
        grip_pos = self.last_obs["observation"][0:3]

        while np.linalg.norm(goal - grip_pos) >= 0.01 and self.time_step <= self.env._max_episode_steps:

            action = [0, 0, 0, 0]
            for i in range(len(goal - grip_pos)):
                action[i] = (goal - grip_pos)[i] * 10 + self.system_noise_scale * np.random.normal()
            action[-1] = 0.05

            self._step(action)

            grip_pos = self.last_obs["observation"][0:3]

    def _stay(self):
        while self.time_step <= self.env._max_episode_steps:

            action = [0, 0, 0, 0]
            for i in range(3):
                action[i] = self.system_noise_scale * np.random.normal()
            action[-1] = 0.05  # keep the gripper open

            self._step(action)

def main():

    # Change the following parameters
    num_itr = 10
    render = True
    env_name = "FrankaPegInHole"
    env = EnvManager(env_name=env_name, env_args={}, r_scale=1.0, r_shift=0.0, eps_length=50).get_env()
    system_noise_scale = 0.00
    # Eps length to use:
    #   FetchPickAndPlace: eps = 50
    #   FetchMoveAutoPlace: 2 objects eps = 80
    #   FetchMove: 2 objects eps = 70
    # End

    generator = FetchPickAndPlaceDemoGenerator(env=env, system_noise_scale=system_noise_scale, render=render)
    demo_data_obs = []
    demo_data_acs = []
    demo_data_rewards = []
    demo_data_info = []

    for i in range(num_itr):
        print("Iteration number: ", i)

        episode_obs, episode_act, episode_rwd, episode_info = generator.generate_peg_in_hole()
        demo_data_obs.append(episode_obs)
        demo_data_acs.append(episode_act)
        demo_data_rewards.append(episode_rwd)
        demo_data_info.append(episode_info)

    T = env._max_episode_steps
    result = None
    for epsd in range(num_itr):  # we initialize the whole demo buffer at the start of the training
        obs, acts, goals, achieved_goals, rs = [], [], [], [], []
        info_keys = [key.replace("info_", "") for key in demo_data_info[0][0].keys()]
        info_values = [np.empty((T, 1, 1), np.float32) for key in info_keys]
        for transition in range(T):
            obs.append([demo_data_obs[epsd][transition].get("observation")])
            acts.append([demo_data_acs[epsd][transition]])
            goals.append([demo_data_obs[epsd][transition].get("desired_goal")])
            achieved_goals.append([demo_data_obs[epsd][transition].get("achieved_goal")])
            rs.append([demo_data_rewards[epsd][transition]])
            for idx, key in enumerate(info_keys):
                info_values[idx][transition, 0] = demo_data_info[epsd][transition][key]

        obs.append([demo_data_obs[epsd][T].get("observation")])
        achieved_goals.append([demo_data_obs[epsd][T].get("achieved_goal")])

        episode = dict(o=obs, u=acts, g=goals, ag=achieved_goals, r=rs)
        for key, value in zip(info_keys, info_values):
            episode["info_{}".format(key)] = value

        # switch to batch major
        episode_batch = {}
        for key in episode.keys():
            val = copy(np.array(episode[key]))
            # make inputs batch-major instead of time-major
            episode_batch[key] = val.swapaxes(0, 1)
        episode = episode_batch

        if result == None:
            result = episode
        else:
            for k in result.keys():
                result[k] = np.concatenate((result[k], episode[k]), axis=0)

    # array(batch_size x (T or T+1) x dim_key), we only need the first one!
    np.savez_compressed("/home/melissa/Workspace/RLProject/Experiment/ExpData/FrankaPegInHoleDemoData/demo_data.npz", **result)  # save the file


if __name__ == "__main__":
    main()

# Franka env

# env_manager = EnvManager("FrankaPegInHole")
# panda_robot = env_manager.get_env()

# # panda_robot = FrankaPegInHole()    
# actions = (
#     [-1.0, -1.0, -1.0],
#     [+1.0, -1.0, -1.0],
#     [+1.0, +1.0, -1.0],
#     [-1.0, +1.0, -1.0],
#     [-1.0, -1.0, +1.0],
#     [+1.0, -1.0, +1.0],
#     [+1.0, +1.0, +1.0],
#     [-1.0, +1.0, +1.0],
# )

# counter = 0
# panda_robot.reset()
# while not rospy.is_shutdown():
#     for action in actions:
#         for _ in range(50):
#             obs = panda_robot.step(action)
#             # print(obs)
#         panda_robot.reset()

# panda_robot.env.disable_vel_control()
# panda_robot.env.enable_pos_control()
# panda_robot.env.panda_client.go_home()
# panda_robot.env.disable_pos_control()
# panda_robot.env.enable_vel_control()