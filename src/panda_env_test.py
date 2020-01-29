import gym
import numpy as np


if __name__ == '__main__':
    env = gym.make('gym_panda_robot:panda-robot-reacher-v1')

    v = np.array([1.0, 1.0, 1.0])
    
    for epoch in range(5):
        env.reset()

        # switch the direction of the desired end effector velocity
        v[epoch % 3] = -v[epoch % 3]
        
        for t in range(100):
            action = np.array([0.0, 0.0, 0.0])
            action[epoch % 3] = v[epoch % 3]
            obs, reward, done, info = env.step(action)
        print(obs)
            
        
        
