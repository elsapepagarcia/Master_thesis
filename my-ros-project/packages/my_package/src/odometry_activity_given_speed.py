from typing import Tuple

import numpy as np

path_points = np.array([[1.0, 0.00, 1.0],[1.0, 0.00, 0.0],[0.0, 0.00, 0.0],[0.0, 0.00, 1.0]])
  
path_point = path_points[1]
i = 1
flag = 0
time = 0

def delta_phi(ticks: int, prev_ticks: int, resolution: int) -> Tuple[float, float]:
    """
    Args:
        ticks: Current tick count from the encoders.
        prev_ticks: Previous tick count from the encoders.
        resolution: Number of ticks per full wheel rotation returned by the encoder.
    Return:
        dphi: Rotation of the wheel in radians.
        ticks: current number of ticks.
    """

    delta_ticks = ticks-prev_ticks

    # Assuming no wheel slipping
    dphi = 2*np.pi*delta_ticks/resolution


    return dphi, ticks


def pose_estimation(
    R: float,
    baseline: float,
    x_prev: float,
    y_prev: float,
    theta_prev: float,
    delta_phi_left: float,
    delta_phi_right: float,
) -> Tuple[float, float, float]:

    """
    Calculate the current Duckiebot pose using the dead-reckoning model.

    Args:
        R:                  radius of wheel (both wheels are assumed to have the same size) - this is fixed in simulation,
                            and will be imported from your saved calibration for the real robot
        baseline:           distance from wheel to wheel; 2L of the theory
        x_prev:             previous x estimate - assume given
        y_prev:             previous y estimate - assume given
        theta_prev:         previous orientation estimate - assume given
        delta_phi_left:     left wheel rotation (rad)
        delta_phi_right:    right wheel rotation (rad)

    Return:
        x_curr:                  estimated x coordinate
        y_curr:                  estimated y coordinate
        theta_curr:              estimated heading
    """

    # x_curr = x_prev + R*(delta_phi_left+delta_phi_right)*np.cos(theta_prev)/2
    # y_curr = y_prev + R*(delta_phi_left+delta_phi_right)*np.sin(theta_prev)/2
    # theta_curr = theta_prev + R*(delta_phi_right-delta_phi_left)/(baseline)

    global path_points, path_point, i, flag, time
   
    w = [R, 2*R / baseline, 1]

    x = np.array(
        [
            [
                (delta_phi_left + delta_phi_right) * np.cos(theta_prev) / 2,
                (delta_phi_left + delta_phi_right) * np.sin(theta_prev) / 2,
                0,
            ],
            [0, 0, (delta_phi_right - delta_phi_left) / 2],
            [x_prev, y_prev, theta_prev],
        ]
    )
	
    print("This is the theta_prev")
    print(theta_prev)
    print(delta_phi_left)
    print(delta_phi_right)
    print(np.cos(theta_prev))
    print(np.sin(theta_prev))
    print((delta_phi_left + delta_phi_right) * np.cos(theta_prev) / 2)
    print((delta_phi_left + delta_phi_right) * np.sin(theta_prev) / 2)
    x_curr, y_curr, theta_curr = np.array(w).dot(x)
    
    #print("Lidia here, this is the current position:")
    #print(x_curr)
    #print(y_curr)
    #print(theta_curr)
    
    
    cur_pos_x = x_curr
    cur_pos_y = y_curr
    cur_angle = theta_curr#*180/np.pi #+ np.pi/2
    
    #distance = np.sqrt((path_point[0] - cur_pos_x)**2 + (path_point[2] - cur_pos_y)**2)
    #target_angle = np.arctan2(path_point[0] - cur_pos_x, path_point[2] - cur_pos_y)
    #if target_angle < 0:
    #	target_angle = 2*np.pi + target_angle
    #angle_error = target_angle - cur_angle
    
    #if distance <= 0.3:
    #	if i < len(path_points):
    #		i = i + 1
    #		if i ==len(path_points):
    #			i = 0
    #		path_point = path_points[i] 
    #		target_angle = np.arctan2(path_point[0] - cur_pos_x, path_point[2] - cur_pos_y)
    #		if target_angle < 0:
    #			target_angle = 2*np.pi + target_angle
    #		angle_error = target_angle - cur_angle
    #		distance = np.sqrt((path_point[0] - cur_pos_x)**2 + (path_point[2] - cur_pos_y)**2)
    #if target_angle >= 0:
    #	if (cur_angle - np.pi) > target_angle:
    #		angle_error= 2*np.pi - cur_angle + target_angle  
    
    #if i == 0:
    #	ka = 0
    #	kp = 0
    #else:
    ka = 0.2
    kp = 0
    
    #v = ka * distance
    v = ka
    #omega = kp * angle_error
    omega = kp
    '''
    if (path_point[0] == 1) & (path_point[2] == 0.0):
    	
    	v = ka
    	omega = 0
    
    elif (path_point[0] == 0) & (path_point[2] == 0.0):
    	
    	v = ka
    	omega = np.pi/2
    	if flag == 1:
    		v = ka
    		omega = 0
    		time = 1
    	flag = 1
    elif (path_point[0] == 0) & (path_point[2] == 1.0):
    	
    	v = ka
    	omega = np.pi/2
    	if flag == 2:
    		v = ka
    		omega = 0
    		time = 1
    	flag = 2
    	
    elif (path_point[0] == 1) & (path_point[2] == 1.0):
    	
    	v = ka
    	omega = np.pi/2
    	if flag == 3:
    		v = ka
    		omega = 0
    		time = 1
    	flag = 3
    
    if time >=1:
    	v = ka
    	omega = np.pi/2
    	if time == 30:
    	   time = 0
    	time = time + 1'''
    
    action = np.array([v, omega])

    print("This is the current position:")
    print(x_curr)
    print(y_curr)
    print(theta_curr*180/np.pi)
    print(theta_curr)
    print("Increment:")
    print(x_curr - x_prev)
    print(y_curr - y_prev)
    #print("This is the target angle:")
    #print(target_angle)
    #print("This is the angle error:")
    #print(angle_error)
    #print("This is the next path point:")
    #print(path_point)
    #print("This is the omega speed:")
    #print(omega)

    return x_curr, y_curr, theta_curr, v, omega
