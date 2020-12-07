---
layout: post
title: Timing is everything!
subtitle: How do you teach a neural net about time?
#cover-img: /assets/img/path.jpg
#thumbnail-img: /assets/img/thumb.png
#share-img: /assets/img/path.jpg
---


![Dataset Diagram](/ml_comprobofinal/img/dataset_diagram.png){: .mx-auto.d-block :}
![Dataset collection in gazebo](/ml_comprobofinal/img/training_data.gif){: .mx-auto.d-block :}

One of the potential limitations of our first model is that it only looks at the current positions of the balls in the world. 
This is a tricky problem, most simple Neural Net models are made to perform on a single timestep’s information.


Our first thought was to use some form of a convolutional neural network to utilize their power to convolute in time.
Convolutional layers of neural networks are used lots in image processing where features are located throughout the frame.
The use of convolutional layers helps the network generalize and be less location dependent.
We are trying to employ a similar practice here in which we convolute throughout time (whereas images usually convolute in 2 dimensions). 

Further research into this idea brought us to a popular style of neural networks.
They are called Long Short-Term Memory Networks -- or LSTM for short.
These networks convolute in time and are very commonly used in a field of ML related to time series forecasting. 

This format lends itself exactly to our data: we have data points at a reproducible time step that describe the world that the robot is in.
This data includes the positions of the balls and their trajectories. We create training data by piloting the robot ourselves in our simulation.
This means that we have data that corresponds to the state of the world at a time stamp and the motor command that is being applied.
When using a LSTM model we simply group a certain amount of timesteps together(how far into the past the model looks), we have been using 5 time steps.
This model then generates the output motor command.

The great news about this is that we simply have to do a little data wrangling to change the outputs that are stored for every timestep and put them into this format.
We used tensorflow to get this working (their was more documentation and examples [https://www.tensorflow.org/tutorials/structured_data/time_series] )

Several baseline models were trained. 
These revolved around the robot dodging only straight balls, balls at random angles, and balls that always targeted the neato.
All of these models showed some degree of promise although they didn’t successfully dodge balls. Please take a look at them below.

![Striaght Model](/ml_comprobofinal/img/blog_straight.gif){: .mx-auto.d-block :}
![Random](/ml_comprobofinal/img/blog_random.gif){: .mx-auto.d-block :}
![Neato](/ml_comprobofinal/img/blog_neato.gif){: .mx-auto.d-block :}

As you can see, sometimes the model seems to make very good choices and sometimes very poor ones.
I expect that this is due to the fact that we have a relatively small amount of training data and that we are manufacturing the training data.
By manufacturing I mean that from time step to time step the data in a given spot could be referencing different balls.
This is likely problematic when convoluting in time.

The problems above can hopefully be addressed by creating more training data and ensuring that balls stay in the same position each time step.

Next steps involve looking at optimizing this model for better performance with more data, improving the data layout.
Alternatively, pivots could be made to another machine learning algorithm or strategy like reinforcement learning.

Check back again to see where this ends!
