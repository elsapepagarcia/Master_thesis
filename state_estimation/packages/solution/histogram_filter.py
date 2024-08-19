# start by importing some things we will need
import numpy as np
import os
from math import floor, sqrt
from scipy.ndimage.filters import gaussian_filter
from scipy.stats import multivariate_normal

global x_g
global y_g
global ang_g
#global xy_array
#global ang_array 
x_g = 0
y_g = 0
ang_g = 0
#xy_array = np.array([])
#ang_array = np.array([])

# Now let's define the prior function. In this case we choose
# to initialize the historgram based on a Gaussian distribution around [0,0]
def histogram_prior(belief, grid_spec, mean_0, cov_0):
    pos = np.empty(belief.shape + (2,))
    pos[:, :, 0] = grid_spec["d"]
    pos[:, :, 1] = grid_spec["phi"]
    RV = multivariate_normal(mean_0, cov_0)
    belief = RV.pdf(pos)
    return belief


# Now let's define the predict function

def histogram_predict(belief, left_encoder_ticks, right_encoder_ticks, grid_spec, robot_spec, cov_mask):

    global x_g
    global y_g
    global ang_g

    belief_in = belief.copy()  # Make a copy of the input belief to work with

    # Calculate v and w from ticks using kinematics
    ticks_to_meters = 2 * np.pi * robot_spec['wheel_radius'] / robot_spec['encoder_resolution']
    v_left = left_encoder_ticks * ticks_to_meters
    v_right = right_encoder_ticks * ticks_to_meters
    v = (v_left + v_right) / 2
    w = (v_right - v_left) / robot_spec['wheel_baseline']
    
    #print("This is w")
    #print(w)
    #print("This is the cosine")
    #print(np.cos(ang_g))
    #print("This is the sine")
    #print(np.sin(ang_g))
    #print("This is the distance of the left:")
    #print(v_left)
    #print("This is the distance of the right:")
    #print(v_right)
    #print("This is the distance covered:")
    #print(v)

    #xy_array = np.append(xy_array, [x_g, y_g])
    #ang_g = np.append(ang_array, ang_g)

    # KEEP THIS #
    x_g = x_g + v*np.cos(ang_g)
    y_g = y_g + v*np.sin(ang_g)
    ang_g = ang_g + w

    #print("These are the parameters:")
    #print(x_g)
    #print(y_g)
    #print(ang_g)

    # Find the current best heading estimate
    maxids = np.unravel_index(belief_in.argmax(), belief_in.shape)
    phi_max = grid_spec['phi_min'] + (maxids[1] + 0.5) * grid_spec['delta_phi']

    # Propagate each centroid
    d_t = grid_spec['d'] + v * np.cos(phi_max + grid_spec['phi'])
    phi_t = grid_spec['phi'] + w

    p_belief = np.zeros(belief.shape)

    # Accumulate the mass for each cell as a result of the propagation step
    for i in range(belief.shape[0]):
        for j in range(belief.shape[1]):
            # If belief_in[i, j] there was no mass to move in the first place
            if belief_in[i, j] > 0:
                # Now check that the centroid of the cell wasn't propagated out of the allowable range
                if (d_t[i, j] > grid_spec['d_max'] or d_t[i, j] < grid_spec['d_min'] or phi_t[i, j] < grid_spec['phi_min'] or phi_t[i, j] > grid_spec['phi_max']):
                    continue

                # Find the cell where the new mass should be added
                i_new = np.floor((d_t[i, j] - grid_spec['d_min']) / grid_spec['delta_d']).astype(int)
                j_new = np.floor((phi_t[i, j] - grid_spec['phi_min']) / grid_spec['delta_phi']).astype(int)

                p_belief[i_new, j_new] += belief_in[i, j]

    # Finally, add some "noise" according to the process model noise
    s_belief = np.zeros(belief.shape)
    gaussian_filter(p_belief, cov_mask, output=s_belief, mode='constant')

    # Normalize the resulting belief
    if np.sum(s_belief) > 0:
        belief = s_belief / np.sum(s_belief)
    else:
        belief = s_belief
    
    #with open("./output.txt", "a") as file:
    #    file.write("\n" + str(x_g) + ", " + str(x_g) + ", " + str(ang_g))
    # Get the current working directory
    #current_directory = os.getcwd()

    # Print the current working directory
    #print(f"The current directory is: {current_directory}")
    
    # ALSO KEEP THIS #
    print("\n" + str(x_g) + ", " + str(y_g) + ", " + str(ang_g))


    return belief


# We will start by doing a little bit of processing on the segments to remove anything that is
# behing the robot (why would it be behind?) or a color not equal to yellow or white


def prepare_segments(segments, grid_spec):
    filtered_segments = []
    for segment in segments:

        # we don't care about RED ones for now
        if segment.color != segment.WHITE and segment.color != segment.YELLOW:
            continue
        # filter out any segments that are behind us
        if segment.points[0].x < 0 or segment.points[1].x < 0:
            continue

        point_range = getSegmentDistance(segment)
        if grid_spec["range_est"] > point_range > 0:
            filtered_segments.append(segment)
    return filtered_segments


def generate_vote(segment, road_spec):
    p1 = np.array([segment.points[0].x, segment.points[0].y])
    p2 = np.array([segment.points[1].x, segment.points[1].y])
    t_hat = (p2 - p1) / np.linalg.norm(p2 - p1)
    n_hat = np.array([-t_hat[1], t_hat[0]])

    d1 = np.inner(n_hat, p1)
    d2 = np.inner(n_hat, p2)
    l1 = np.inner(t_hat, p1)
    l2 = np.inner(t_hat, p2)
    if l1 < 0:
        l1 = -l1
    if l2 < 0:
        l2 = -l2

    l_i = (l1 + l2) / 2
    d_i = (d1 + d2) / 2
    phi_i = np.arcsin(t_hat[1])
    if segment.color == segment.WHITE:  # right lane is white
        if p1[0] > p2[0]:  # right edge of white lane
            d_i -= road_spec["linewidth_white"]
        else:  # left edge of white lane
            d_i -= road_spec["linewidth_white"]
            d_i = road_spec["lanewidth"] * 2 + road_spec["linewidth_yellow"] - d_i
            phi_i = -phi_i
        d_i -= road_spec["lanewidth"]/2

    elif segment.color == segment.YELLOW:  # left lane is yellow
        if p2[0] > p1[0]:  # left edge of yellow lane
            d_i -= road_spec["linewidth_yellow"]
            d_i = road_spec["lanewidth"]/2 - d_i
            phi_i = -phi_i
        else:  # right edge of yellow lane
            d_i += road_spec["linewidth_yellow"]
            d_i -= road_spec["lanewidth"]/2

    return d_i, phi_i

def generate_measurement_likelihood(segments, road_spec, grid_spec):
    # initialize measurement likelihood to all zeros
    measurement_likelihood = np.zeros(grid_spec["d"].shape)

    for segment in segments:
        d_i, phi_i = generate_vote(segment, road_spec)

        # if the vote lands outside of the histogram discard it
        if (
            d_i > grid_spec["d_max"]
            or d_i < grid_spec["d_min"]
            or phi_i < grid_spec["phi_min"]
            or phi_i > grid_spec["phi_max"]
        ):
            continue

        # Convert d_i and phi_i to grid indices
        i = int((d_i - grid_spec["d_min"]) / grid_spec["delta_d"])
        j = int((phi_i - grid_spec["phi_min"]) / grid_spec["delta_phi"])

        # Add one vote to that cell
        measurement_likelihood[i, j] += 1

    if np.linalg.norm(measurement_likelihood) == 0:
        return None
    measurement_likelihood /= np.sum(measurement_likelihood)
    return measurement_likelihood
    
def histogram_update(belief, segments, road_spec, grid_spec):
    # prepare the segments for each belief array
    segmentsArray = prepare_segments(segments, grid_spec)
    
    # generate measurement likelihood from segments
    measurement_likelihood = generate_measurement_likelihood(segmentsArray, road_spec, grid_spec)

    if measurement_likelihood is not None:
        # Combine the prior belief and the measurement likelihood to get the posterior belief
        belief *= measurement_likelihood
        
        
        belief_sum = np.sum(belief)
        if belief_sum > 0:
            belief /= belief_sum
        else:
            # Handle the case where the belief sum is zero (e.g., by returning the original belief or setting to a default distribution)
            print("Warning: Sum of belief is zero. Returning unmodified belief.")
            # Optionally, you can reset the belief to a prior or uniform distribution
            # belief = np.ones_like(belief) / belief.size  # Example: uniform distribution


    return measurement_likelihood, belief

def getSegmentDistance(segment):
    x_c = (segment.points[0].x + segment.points[1].x) / 2
    y_c = (segment.points[0].y + segment.points[1].y) / 2
    return sqrt(x_c**2 + y_c**2)
