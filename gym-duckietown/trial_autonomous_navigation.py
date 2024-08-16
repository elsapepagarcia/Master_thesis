#!/usr/bin/env python
# manual


import numpy as np
from gym_duckietown.envs.duckietown_env import DuckietownNav
from gym_duckietown.simulator import Simulator

# Initialize the environment
env = DuckietownNav(map_name='small_loop', domain_rand=False)

# Reset the environment to get the initial observation
obs = env.reset()

env.reset()
env.cur_pos = np.array([1.25, 0, 1.58])
#np.array([0.45, 0, 0.54])
env.cur_angle = 0
env.render()

# Define a simple random policy for navigation
def random_policy():
    # Generate random actions within the action space
    action = env.action_space.sample()
    return action

# Main navigation loop
done = False
total_reward = 0

while not done:
    # Get an action from the policy
    action = random_policy()
    
    # Take a step in the environment
    obs, reward, done, info = env.step(action)
    
    # Accumulate reward
    total_reward += reward
    
    # Print the current state information
    print(f"Current Position: {env.cur_pos}, Goal Tile: {info['goal_tile']}")
    print(f"Reward: {reward}, Total Reward: {total_reward}")
    env.render()

print("Navigation completed!")

