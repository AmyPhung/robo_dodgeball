"""
This provides a GUI interface so you can artifically put in whatever
inputs you want to the network to visualize its response to arbitrary cases

KEY ASSUMPTION ABOUT X Y COORDINATES:
(D represents our neato)

               +y
               ^
               |
       -x <--  D  --> +x
               |
               v
               -y
"""

import cv2
import numpy as np
from copy import deepcopy
from helpers import get_ws_path
import torch
from torch.autograd import Variable  # Data class for nn training and testing

# Define global variables

# Each meter is 100 pixels
PIXEL_TO_METER_RATIO = 50

side_len_x = 10.0 # m
side_len_y = 10.0 # m
max_ball_velocity = 5.0 # m/s

SIDE_LEN_X_P = int(side_len_x * PIXEL_TO_METER_RATIO)
SIDE_LEN_Y_P = int(side_len_y * PIXEL_TO_METER_RATIO)

background_screen = np.ones((SIDE_LEN_Y_P, SIDE_LEN_X_P, 3))*0

def nothing(x):
    pass

def computeBallDistance(ball_x_m, ball_y_m, neato_x_m):
  return np.sqrt( (ball_x_m - neato_x_m)**2 + (ball_y_m)**2 )

def computeBallAngle(ball_x_m, ball_y_m, ball_vx_ms, ball_vy_ms, neato_x_m):

  r_pos_vec = np.array([neato_x_m, 0])
  b_pos_vec = np.array([ball_x_m, ball_y_m])
  b_vel_vec = np.array([ball_vx_ms, ball_vy_ms])

  # Compute the angle between the ball trajectory and direction to robot
  vec_to_robot = r_pos_vec - b_pos_vec
  vec_to_robot, b_vel_vec

  # If ball isn't moving, return pi
  if (b_vel_vec[0]==0 and b_vel_vec[1]==0):
      return np.pi

  unit_vec_to_robot = vec_to_robot / np.linalg.norm(vec_to_robot)
  unit_b_vel_vec = b_vel_vec / np.linalg.norm(b_vel_vec)
  dot_product = np.dot(unit_vec_to_robot, unit_b_vel_vec)
  angle = np.arccos(dot_product)

  # Append sign to calculation
  cross_product = np.cross(vec_to_robot, b_vel_vec)
  if (cross_product < 0):
      angle = -angle

  return angle

def computeBallVelocity(ball_vx_ms, ball_vy_ms):
  b_vel_vec = np.array([ball_vx_ms, ball_vy_ms])
  vel_magnitude = np.linalg.norm(b_vel_vec)

  return vel_magnitude

def meters_to_pixels(m):
  """ This takes in a pixel measurement and outputs a meter measurement"""
  try:
    return int(m * PIXEL_TO_METER_RATIO)
  except ValueError:
    return 0

def mcoords_to_pcoords(x_m,y_m):
  """ This converts from an x, y in meter coordinates to an x, y in pixel coordinates """
  # Convert meters to pixels
  x_p = meters_to_pixels(x_m)
  y_p = meters_to_pixels(y_m)

  # Invert the y
  y_p = SIDE_LEN_Y_P - y_p

  # Output the new coordinates
  return x_p, y_p

def place_ball(screen, x_m,y_m):
  # Get new coordinates
  x_p, y_p = mcoords_to_pcoords(x_m, y_m)

  # Setup ball
  ball_radius = 0.25 #m
  ball_radius_p = meters_to_pixels(1)

  # Draw a circle on the new screen
  cv2.circle(screen, (x_p,y_p), ball_radius_p, (255,0,0), -1)

  return screen

def place_neato(screen, nx_m):
  # Get new coordinates
  nx_p = meters_to_pixels(nx_m)

  # Setup ball
  neato_radius = 0.1 #m
  neato_radius_p = meters_to_pixels(1)

  # Draw a circle on the new screen
  cv2.circle(screen, (nx_p,SIDE_LEN_Y_P), neato_radius_p, (240,240,240), -1)

  return screen

def place_velocity(screen, init_x_m, init_y_m, vx_m, vy_m):
  # Calculate endpoint for ball in meters
  end_x_m = init_x_m + vx_m
  end_y_m = init_y_m + vy_m

  # Convert meter coords to pixel coords
  init_x_p, init_y_p = mcoords_to_pcoords(init_x_m,init_y_m)
  end_x_p, end_y_p = mcoords_to_pcoords(end_x_m, end_y_m)

  # Create start point and end point
  start_pt = (init_x_p, init_y_p)
  end_pt = (end_x_p, end_y_p)

  # Draw an arrow
  cv2.arrowedLine(screen, start_pt, end_pt, (255,255,255), thickness=3, tipLength=0.3)

  return screen

def update_frame(nx_m, x1_m, y1_m, vx1_ms, vy1_ms, x2_m, y2_m, vx2_ms, vy2_ms):
  """ Updates and displays frame with new data """

  # Get a fresh screen
  new_screen = deepcopy(background_screen)

  # Place the Neato
  new_screen = place_neato(new_screen, nx_m)

  # Place the first ball
  new_screen = place_ball(new_screen, x1_m, y1_m)

  # Place velocity of the first ball
  new_screen = place_velocity(new_screen, x1_m, y1_m, vx1_ms, vy1_ms)

  # Label the first ball
  cv2.putText(new_screen, "1", mcoords_to_pcoords(x1_m-0.25, y1_m-0.25), 0, 2.0, (0,0,0), 3)

  # Place the second ball
  new_screen = place_ball(new_screen, x2_m, y2_m)

  # Place velocity of the second ball
  new_screen = place_velocity(new_screen, x2_m, y2_m, vx2_ms, vy2_ms)

  # Label the second ball
  cv2.putText(new_screen, "2", mcoords_to_pcoords(x2_m-0.25, y2_m-0.25), 0, 2.0, (0,0,0), 3)

  if net:
    # Reformat data for input into net
    # We have 6 features () : 3 features per ball for 2 balls
    #   Distance from robot
    #   Angle of Attack
    #   Magnitude of velocity towards robot
    # and 1 output: v_N : velocity of the Neato

    # Compute input variables for ball 1
    ball1_distance = computeBallDistance(x1_m, y1_m, nx_m)
    ball1_angle = computeBallAngle(x1_m, y1_m, vx1_ms, vy1_ms, nx_m)
    ball1_velocity = computeBallVelocity(vx1_ms, vy1_ms)

    # Compute input variables for ball 2
    ball2_distance = computeBallDistance(x2_m, y2_m, nx_m)
    ball2_angle = computeBallAngle(x2_m, y2_m, vx2_ms, vy2_ms, nx_m)
    ball2_velocity = computeBallVelocity(vx2_ms, vy2_ms)

    # Figure out which ball is closest
    # Package data
    if ball1_distance <= ball2_distance:
      data_pt = np.array([ball1_distance, ball2_distance, ball1_angle, ball2_angle, ball1_velocity, ball2_velocity])
    else:
      data_pt = np.array([ball2_distance, ball1_distance, ball2_angle, ball1_angle, ball2_velocity, ball1_velocity])

    print(data_pt)

    # Convert data to tensor for input into neural net
    input_data = Variable(torch.tensor(data_pt.astype(np.float32)))

    # Get output from network
    output_data = net(input_data)

    # extract the velocity of the neato
    # neato_velocity = input_data.data.numpy()[0] # First ball distance
    neato_velocity = output_data.data.numpy()[0] # Output cmd from model

    print(neato_velocity)

    # Draw the arrow showing what the Neato would do
    cv2.arrowedLine(new_screen, mcoords_to_pcoords(nx_m, 0), mcoords_to_pcoords(nx_m + neato_velocity, 0), (0,200,0), thickness=3, tipLength=0.3)

  # Display the updated screen
#   cv2_imshow(new_screen)

  # Print updated coords
  print(
      "Ball 1 coords:\n"\
      "  x: " + str(x1_m) +\
      "  y: " + str(y1_m) +"\n"\
      "Ball 2 coords:\n"\
      "  x: " + str(x2_m) +\
      "  y: " + str(y2_m)
      )
  return new_screen

def float_to_int(float_value):
    return int(float_value * 100)

def int_to_float(int_value):
    return float(int_value) / 100.0

# Load in the network
model_name = "standard_993"

folder_name = get_ws_path() + "ml_models/" + model_name + "/"
net = torch.load(folder_name + "net.pkl")

# Create a black image, a window
window_name = 'Visualize Cases'
cv2.namedWindow(window_name)
frame = background_screen

# Define variables for trackbars
gui_side_len_x = float_to_int(side_len_x)
gui_side_len_y = float_to_int(side_len_y)
gui_max_ball_velocity = float_to_int(max_ball_velocity)

# Create trackbar for neato position
cv2.createTrackbar('Neato position', window_name, 0,gui_side_len_x,nothing)

# Create trackbar for ball 1 position and velocity
cv2.createTrackbar('Ball 1 x position', window_name,0,gui_side_len_x,nothing)
cv2.createTrackbar('Ball 1 y position', window_name,0,gui_side_len_x,nothing)
cv2.createTrackbar('Ball 1 x velocity', window_name,0,gui_max_ball_velocity*2,nothing)
cv2.createTrackbar('Ball 1 y velocity', window_name,0,gui_max_ball_velocity*2,nothing)

# Create trackbar for ball 2 position and velocity
cv2.createTrackbar('Ball 2 x position', window_name,0,gui_side_len_x,nothing)
cv2.createTrackbar('Ball 2 y position', window_name,0,gui_side_len_y,nothing)
cv2.createTrackbar('Ball 2 x velocity', window_name,0,gui_max_ball_velocity*2,nothing)
cv2.createTrackbar('Ball 2 y velocity', window_name,0,gui_max_ball_velocity*2,nothing)

while(1):
    cv2.imshow(window_name,frame)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

    # get current positions of all trackbars

    nx_m = int_to_float(cv2.getTrackbarPos('Neato position', window_name))

    x1_m = int_to_float(cv2.getTrackbarPos('Ball 1 x position', window_name))
    y1_m = int_to_float(cv2.getTrackbarPos('Ball 1 y position', window_name))
    vx1_ms = int_to_float(cv2.getTrackbarPos('Ball 1 x velocity', window_name)) - max_ball_velocity
    vy1_ms = int_to_float(cv2.getTrackbarPos('Ball 1 y velocity', window_name)) - max_ball_velocity

    x2_m = int_to_float(cv2.getTrackbarPos('Ball 2 x position', window_name))
    y2_m = int_to_float(cv2.getTrackbarPos('Ball 2 y position', window_name))
    vx2_ms = int_to_float(cv2.getTrackbarPos('Ball 2 x velocity', window_name)) - max_ball_velocity
    vy2_ms = int_to_float(cv2.getTrackbarPos('Ball 2 y velocity', window_name)) - max_ball_velocity

    frame = update_frame(nx_m, x1_m, y1_m, vx1_ms, vy1_ms, x2_m, y2_m, vx2_ms, vy2_ms)

cv2.destroyAllWindows()

