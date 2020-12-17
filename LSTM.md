---
layout: page
title: LSTM
subtitle: Long Short Term Memory Network
---

#### LSTM Crash Course / Application
__**[See this blog post to read more about why we were interested in LSTM models](https://everardog.github.io/ml_comprobofinal/2020-12-06-timing-is-everything/)**__

####  Training
The LSTM model requires a unique format of input data to train. If you remember correctly, we have time series data that contains motor commands,  ball positions and velocities at a given time. The LSTM model takes in a chunk's worth of time to predict ONE output. In our case the chunk of time will include the ball positions and velocities. And we will be working to predict motor commands.

This data transformation can be done quite simply. The large time series is simply grouped into consecutive batches. For example, we start with a raw data blob that is 9 columns across (4 for each ball and 1 for the motor command) and T timesteps long __shape = [T, 9]__. If an LSTM network that utilizes N previous timesteps is being trained this data would be parsed into two arrays of __shape [T-N, N, 8]__ and __shape [T-N, 1]__ representing the model inputs and outputs respectively.

For our implementation we utilize Tensorflow’s Keras API. We also utilize the pre-built LSTM layer from Keras [LSTM Layer Document](https://www.tensorflow.org/api_docs/python/tf/keras/layers/LSTM). This is somewhat of a “shortcut”, however, we looked into a manual implementation of an LSTM layer here, but for our purposes we decided that it made more sense to use the prebuilt one.

#### Choosing LSTM Parameters
As mentioned earlier, there are several parameters of an LSTM network that can be tuned.
There are 2 parameters available to us, the number of timesteps to look backward and the number of “neurons” within the LSTM layer. We chose these values via some educated guessing, as well as some testing. For example the balls spawn roughly 5m away from the robot and can move @5m/s. We have a slow loop time of 10hz that means we sample positions and apply the model at that rate. This means that we should never look at  more than 10 time steps since that would be a full movement of the ball. We likely want a number smaller than that so that the model can be applied correctly at different parts of the ball’s path. (ie a ten time step model may work well at predicting what to do when we have close to 10 time steps of data on that ball, but it will likely be too late for the robot to move).

Tuning the “neurons” of the LSTM layer is much more subjective. This parameter should be scaled relatively to the complexity of your problem. Based on articles that we found this value could range anywhere from 5-200. We tested a variety of different values, and compared loss metrics during the training of the neural net to determine the right parameter to use. 
The easiest thing to tell is if you do not have enough “neurons” in this case, the loss for both the train and eval sets will flatten out at a relatively high value. This indicates that the network can’t learn the more complex behavior/trends shown within the dataset.

Below is a subset of training logs showing different combinations of parameters training our model on the same data.

![Dataset Diagram](/ml_comprobofinal/img/LSTM_testing.svg)

We found that many of the models with 15-20 hidden neurons ended up being overfit. Models with fewer than 8 ended up not being able to capture the complexity of the problem. 

Based solely on these loss plots, it appears that 5-8 timesteps and 15 neurons would yield some of the best models. 

__Robot running via ML using a 5 timestep and 15 neuron network__

![LSTM 5_15](/ml_comprobofinal/img/LSTM_05_15.gif)

#####__[View our blog post to see our various models running!](https://everardog.github.io/ml_comprobofinal/2020-12-15-LSTM_models/)__
