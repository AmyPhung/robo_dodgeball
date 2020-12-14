"""
Class definition for neural network

Assumptions about data:
  We have 4 features () :
    4 features four our 1 ball
        px,py position of ball relative to neato
        vx,vy velocity of ball relative to neato

  and 1 output: v_N : x velocity of the Neato

  In vector form: [v_N px1 py1 vx1 vy1]
  Bias term is also encoded into our net
"""

import torch
from torch import nn

class OneBallNN(nn.Module):
  def __init__(self, learning_rate=0.1):
    # Runs the init method from the nn.Module class as though OneBallNN was an nn.Module
    super(OneBallNN, self).__init__()

    # Defining an activation function
    # Relu turns any negative input to 0, so not what we want
    # Sigmoid makes any negative input positive, so no
    # Tanh is sigmoid, but centered around 0
    self.activation_func = torch.nn.Tanh()

    # We have 10 features () : 5 features per ball for 2 balls
    #   Distance from robot
    #   x,y velocity of ball in robot frame
    #   x,y position of ball in robot frame
    # and 1 output: v_N : velocity of the Neato
    # Bias term is also encoded into our net

    # Now we set up our layers
    input_size = 4
    fc1_size = 2
    fc2_size = 1

    # fc stands for Fully Connected Layer
    self.fc1 = nn.Linear(input_size, fc1_size)
    self.fc2 = nn.Linear(fc1_size, fc2_size)

    # Setup loss and optimizer functions
    self.lossFunction = nn.MSELoss()

    # self.parameters() returns all the Pytorch operations that are attributes of the class
    self.optimizerFunction = torch.optim.Adam(self.parameters(), lr=learning_rate)

  def forward(self, x):
    # Run training data through first fc layer
    x = self.fc1(x)

    # Run data through activation function
    x = self.activation_func(x)

    # Run data through second fc layer
    x = self.fc2(x)

    # Run data through activation function
    x = self.activation_func(x)

    # Final data should be 1x1
    # Multiply data by maximum magnitude of input velocity
    #   since our activation function maps from -1 to 1
    max_speed = 0.5
    x = max_speed * x

    return x
