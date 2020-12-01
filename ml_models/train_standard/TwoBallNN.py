"""
Class definition for neural network

Assumptions about data:
  We have 6 features () : 3 features per ball for 2 balls
    Distance from robot
    Angle of Attack
    Magnitude of velocity towards robot
  and 1 output: v_N : velocity of the Neato
  Bias term is also encoded into our net
"""

import torch
from torch import nn

class TwoBallNN(nn.Module):
  def __init__(self, learning_rate=0.1):
    # Runs the init method from the nn.Module class as though TwoBallNN was an nn.Module
    super(TwoBallNN, self).__init__()

    # Defining an activation function
    # Relu turns any negative input to 0, so not what we want
    # Sigmoid makes any negative input positive, so no
    # Tanh is sigmoid, but centered around 0
    self.activation_func = torch.nn.Tanh()

    # We have 6 features () : 3 features per ball for 2 balls
    #   Distance from robot
    #   Angle of Attack
    #   Magnitude of velocity towards robot
    # and 1 output: v_N : velocity of the Neato
    # Bias term is also encoded into our net

    # Now we set up our layers
    input_size = 6
    fc1_size = 4
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
