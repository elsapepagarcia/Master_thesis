from typing import Tuple

import numpy as np

path_points = np.array([[1.40, 0.00, 1.40], [1.40, 0.00, 1.0],[1.40, 0.00, 0.30],[0.85, 0.00, 0.30],[0.40, 0.00, 0.30],  [0.40, 0.00, 1.0],[0.40, 0.00, 1.40],[0.85, 0.00, 1.40]])
  
path_point = path_points[1]
i = 1

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

    global path_points, path_point, i
   
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

    x_curr, y_curr, theta_curr = np.array(w).dot(x)
    
    print("Lidia here, this is the current position:")
    print(x_curr)
    print(y_curr)
    print(theta_curr)
    
    
    cur_pos_x = x_curr
    cur_pos_y = y_curr
    cur_angle = theta_curr
    
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
    
    action = np.array([v, omega])


    return x_curr, y_curr, theta_curr
