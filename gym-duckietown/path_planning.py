#!/usr/bin/env python
# manual

"""
This script allows you to manually control the simulator or Duckiebot
using the keyboard arrows.
"""

import sys
import numpy as np

np_int = np.int32
import argparse
import pyglet
from pyglet.window import key
import numpy as np
import gym
import gym_duckietown
from gym_duckietown.envs import DuckietownEnv
from gym_duckietown.wrappers import UndistortWrapper

# from experiments.utils import save_img

parser = argparse.ArgumentParser()
parser.add_argument('--env-name', default=None)
parser.add_argument('--map-name', default='small_loop')
parser.add_argument('--distortion', default=False, action='store_true')
parser.add_argument('--draw-curve', action='store_true', help='draw the lane following curve')
parser.add_argument('--draw-bbox', action='store_true', help='draw collision detection bounding boxes')
parser.add_argument('--domain-rand', action='store_true', help='enable domain randomization')
parser.add_argument('--frame-skip', default=1, type=int, help='number of frames to skip')
parser.add_argument('--seed', default=1, type=int, help='seed')
args = parser.parse_args()

if args.env_name and args.env_name.find('Duckietown') != -1:
    env = DuckietownEnv(
        seed = args.seed,
        map_name = args.map_name,
        draw_curve = args.draw_curve,
        draw_bbox = args.draw_bbox,
        domain_rand = args.domain_rand,
        frame_skip = args.frame_skip,
        distortion = args.distortion,
    )
else:
    env = gym.make(args.env_name)

env.reset()
env.cur_pos = np.array([1.40, 0.00, 1.57])
env.cur_angle = np.pi/2
env.render()

cur_pos_print = np.array([1.40, 0.00, 1.57])


@env.unwrapped.window.event
def on_key_press(symbol, modifiers):
    """
    This handler processes keyboard commands that
    control the simulation
    """

    if symbol == key.BACKSPACE or symbol == key.SLASH:
        print('RESET')
        env.reset()
        env.render()
    elif symbol == key.PAGEUP:
        env.unwrapped.cam_angle[0] = 0
    elif symbol == key.ESCAPE:
        env.close()
        sys.exit(0)

    # Take a screenshot
    # UNCOMMENT IF NEEDED - Skimage dependency
    # elif symbol == key.RETURN:
    #     print('saving screenshot')
    #     img = env.render('rgb_array')
    #     save_img('screenshot.png', img)

# Register a keyboard handler
key_handler = key.KeyStateHandler()
env.unwrapped.window.push_handlers(key_handler)

path_points = np.array([[1.40, 0.00, 1.40], [1.40, 0.00, 1.0],[1.40, 0.00, 0.30],[0.85, 0.00, 0.30],[0.40, 0.00, 0.30],  [0.40, 0.00, 1.0],[0.40, 0.00, 1.40],[0.85, 0.00, 1.40]])
  
path_point = path_points[1]
i = 1
env.cur_pos = path_points[0]

def update(dt):
    """
    This function is called at every frame to handle
    movement/stepping and redrawing
    """
    print("This is the current position:")
    print(env.cur_pos)
    #print(env.cur_angle)
    
    global path_points, path_point, i, cur_pos_print
    
    #action = np.array([0.44, 0])
    action = np.array([0, 0.0])
    
    cur_pos_x = env.cur_pos[0]
    cur_pos_y = env.cur_pos[2]
    cur_angle = env.cur_angle + np.pi/2
      
    # First find the closest point in the path
    distance = np.sqrt((path_point[0] - cur_pos_x)**2 + (path_point[2] - cur_pos_y)**2)
    target_angle = np.arctan2(path_point[0] - cur_pos_x, path_point[2] - cur_pos_y)
    if target_angle < 0:
    	target_angle = 2*np.pi + target_angle
    angle_error = target_angle - cur_angle
    
    if distance <= 0.2:
    	if i < len(path_points):
    		i = i + 1
    		if i ==len(path_points):
    			i = 0
    		path_point = path_points[i] 
    		target_angle = np.arctan2(path_point[0] - cur_pos_x, path_point[2] - cur_pos_y)
    		if target_angle < 0:
    			target_angle = 2*np.pi + target_angle
    		angle_error = target_angle - cur_angle
    		distance = np.sqrt((path_point[0] - cur_pos_x)**2 + (path_point[2] - cur_pos_y)**2)
    if target_angle >= 0:
    	if (cur_angle - np.pi) > target_angle:
    		angle_error= 2*np.pi - cur_angle + target_angle  
    
    if i == 0:
    	ka = 0
    	kp = 0
    else:
    	ka = 0.2
    	kp = 0.5
    
    #v = ka * distance
    v = ka
    omega = kp * angle_error
    #print("This is the target_angle")
    #print(target_angle)
    #print("This is the current_angle:")
    #print("Current angle:")
    #print(cur_angle)
    #print("angle_error:")
    #print(angle_error)
    #print("This is the target")
    #print(path_point)
    
    
    action = np.array([v, omega])
    
    # Trying
    #alpha = 2*np.pi / 135
    #time = 1.0 / env.unwrapped.frame_rate
    #ticks = time * 135
    
    #R = 0.0318
    #d = ticks*alpha*R
    #ang = d/0.1
    #cur_pos_print = cur_pos_print + np.array([d*np.sin(ang), 0, d*np.cos(ang)])
    #print("This is the current calculated position:")
    #print(cur_pos_print)
    '''
    
    
    print("distances")
    print(distances)
    print("path point")
    print(path_point)
    if distances <=0.05:
        
    	if i < len(path_points):
    		path_point = path_points[i]
    		i = i + 1
    #closest_point_idx = np.argmin(distances)
    #closest_point = path_points[closest_point_idx]
    closest_point = path_point
    print("This is the closest point:")
    print(closest_point)
        
    # Look ahead to find the target point (the final destiny)
    #lookahead_point_idx = min(closest_point_idx + 5, len(path_points) - 1)
    #lookahead_point = path_points[lookahead_point_idx]
    
    # Compute the target angle
    target_angle = np.arctan2(closest_point[0] - cur_pos_x, closest_point[2] - cur_pos_y)
    print("This is the target_angle:")
    print(target_angle)
    
    # Compute the angle error
    if target_angle < 0:
    	target_angle = 2*np.pi + target_angle
    angle_error = target_angle - (cur_angle + np.pi/2)
    print("This is the angle error:")
    print(angle_error)
    if angle_error > np.pi:
        angle_error -= 2 * np.pi
    elif angle_error < -np.pi:
        angle_error += 2 * np.pi
    print(angle_error)
    # Compute the distance to the target point
    distance = np.sqrt((closest_point[0] - cur_pos_x)**2 + (closest_point[2] - cur_pos_y)**2)

    # Compute control actions
    ka = 0.3
    kp = 1
    v = ka * distance
    omega = kp * angle_error

    action = np.array([v, omega])
    print("This is the action:")
    print(omega)
    '''        
    '''
    if key_handler[key.UP]:
        action = np.array([0.44, 0.0])
    if key_handler[key.DOWN]:
        action = np.array([-0.44, 0])
    if key_handler[key.LEFT]:
        action = np.array([0.35, +1])
    if key_handler[key.RIGHT]:
        action = np.array([0.35, -1])
    if key_handler[key.SPACE]:
        action = np.array([0, 0])
    
    # Speed boost
    if key_handler[key.LSHIFT]:
        action *= 1.5'''
        
    obs, reward, done, info = env.step(action)
    print('step_count = %s, reward=%.3f' % (env.unwrapped.step_count, reward))

    if key_handler[key.RETURN]:
        from PIL import Image
        im = Image.fromarray(obs)

        im.save('screen.png')

    if done:
        print('done!')
        env.reset()
        env.render()

    env.render()

pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

# Enter main event loop
pyglet.app.run()

env.close()
