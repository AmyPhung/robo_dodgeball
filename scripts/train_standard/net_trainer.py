"""
Contains functionality for training the pytorch neural network
"""

import numpy as np                   # Metrics
import logging                       # Organized print statements
logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)

def train_net(net, X_train, y_train, X_test, y_test, num_epochs=1000, learning_rate=0.001, logging_frequency=10):
    """
    Inputs:
        net: pytorch neural network

        X_train: X training data as Variable type
        y_train: y training data as Variable type
        X_test: X testing data as Variable type
        y_test: y testing data as Variable type

        num_epochs: number of epochs to train model for
        learning_rate: how fast the model should be learning
        logging_frequency: log every _ epochs

    Returns:
        (epoch_list, grad_magnitudes, loss_list)
    """
    # Put the model in training mode
    net.train()

    # Set up optimizer for gradient descent
    lossFunction = net.lossFunction
    optimizer = net.optimizerFunction

    grad_magnitudes = []
    loss_list = []
    epoch_list = []

    # Go through all the epochs
    for epoch in range(num_epochs):
        # Clear the gradient
        optimizer.zero_grad()

        # Forward pass input data into
        y_pred = net(X_train)
        loss = lossFunction(y_pred, y_train)
        loss.backward()
        optimizer.step()    # Does the update

        for name, param in net.named_parameters():
            if name == 'fc1.weight':
                grad_magnitudes.append(np.abs(param.grad.numpy()).mean())

        if epoch % logging_frequency == 0:
            logging.info("epoch" + str(epoch))
            for name, param in net.named_parameters():
                logging.info(str(name) + "value" + str(param.data) + "gradient" + str(param.grad))
                logging.info(str(loss))
                epoch_list.append(epoch)
                loss_list.append(loss)

    return epoch_list, grad_magnitudes, loss_list