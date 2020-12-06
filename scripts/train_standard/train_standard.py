"""
Creates and trains a standard neural network for two balls
"""

""" All necessary imports go here """

import os
import torch                         # Main ML Library
import logging                       # Organized print statements
logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
import pickle                        # For saving metrics or meta data

import numpy as np                   # Matrix handling

from sklearn.model_selection import train_test_split
from torch import nn
from torch.autograd import Variable  # Data class for nn training and testing

# Import helper scripts for neural net
from TwoBallNN import TwoBallNN
from net_trainer import train_net

from helpers import get_ws_path

""" Load in our data """

ws_path = get_ws_path()
dataset_loc = ws_path + "datasets/"
dataset_name = "013_2ball_gaussian_1.5x5_vector_joystick_amy.npy"

raw_data = np.load(dataset_loc + dataset_name)

""" Partition data into training and testing data """

raw_data_t = raw_data# raw_data.transpose()

# Organize data into X and y
X_data = raw_data_t[:, 1:]
logging.info("Shape of X data: " + str(X_data.shape))
y_data = raw_data_t[:,0:1]
logging.info("Shape of y data: " + str(y_data.shape))

# Split data into training and testing
X_train , X_test, y_train, y_test = train_test_split(X_data, y_data, test_size = 0.33, random_state = 42)
logging.info("X_train: " + str(type(X_train)) + str(X_train.shape))
logging.info("y_train: " + str(type(y_train)) + str(y_train.shape))
logging.info("X_test:  " + str(type(X_test)) + str(X_test.shape))
logging.info("y_test:  " +str(type(y_test)) + str(y_test.shape))

# Set up data for input into net
X_train = Variable(torch.tensor(X_train.astype(np.float32)))
X_test = Variable(torch.tensor(X_test.astype(np.float32)))
y_train = Variable(torch.tensor(y_train.astype(np.float32)))
y_test = Variable(torch.tensor(y_test.astype(np.float32)))

""" Train the model """

net = TwoBallNN()
epoch_list, grad_magnitudes, loss_list = train_net(net, X_train, y_train, X_test, y_test)

""" Save model and training session info """

model_loc = ws_path + "ml_models/"

# Create a unique id for saving
# TODO: Replace this with appending data the model was trained on

# Create a list of ids that are already taken
file_list = os.listdir(model_loc)
taken_ids = []
for file_ in file_list:
    split_filename = file_.split("_")
    if split_filename[0] == "standard":
        taken_ids.append(split_filename[1])

# Create an id that isn't already taken
temp_id = 999
while str(temp_id) in taken_ids:
    temp_id -= 1

# Save the model and its metrics

# Create a new folder for saving our model
folder_name = model_loc + "standard_" + str(temp_id) + "/"
try:
    os.mkdir(folder_name)
except FileExistsError:
    logging.warn("Folder " + str(folder_name) + " already exists! An old model may be overwritten!")

# Save the model
torch.save(net, folder_name + "net.pkl")

# Save the metrics
metrics_dict = {"epochs": epoch_list, "grad_magnitudes": grad_magnitudes, "loss_list": loss_list}
metrics_file = open(folder_name + "metrics.pkl", 'ab')
pickle.dump(metrics_dict, metrics_file)
metrics_file.close()
