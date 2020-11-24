---
layout: post
title: We've got a dataset
subtitle: Who knows if it's a good one?
#cover-img: /assets/img/path.jpg
#thumbnail-img: /assets/img/thumb.png
#share-img: /assets/img/path.jpg
---

To start off our machine-learning-robo-dodgeball adventure, we first needed a dataset to train our glorious neural net with - and what better way to do that than to collect it in our favorite ROS-compatible simulator, Gazebo!

First, we needed to figure out what data to record. We decided to go with an extremely simple 1-dimensional case where the robot only needs to move back and forth to dodge the balls, which are only coming from one direction. We also decided to pre-process the raw data a bit and distill it to what we believe would be easy to train on just as a gut-check to make sure everything works as expected, and to get our data pipeline up and running. We decided to record the magnitude of the ball's velocity, the distance between the ball and the robot, and the "angle of attack" of the ball rolling towards the robot. In this initial dataset, we recorded these values for two balls, and created "spoof" values if there were no balls in the vicinity of the robot (these values had a 0 velocity, a 1000m distance, and were pointed 180 degrees away from the robot, which we figured should count as the ball the robot should not be worried about at all). At each time step, we also recorded the input command we sent to the robot. All of these values used for our initial dataset can be summarized by the following diagram:

![Dataset Diagram](/img/dataset_diagram.png){: .mx-auto.d-block :}

We then created a ball spawner script that generates balls flying in all directions to collect a dataset with
<!-- TODO: write more about ball spawner -->
![Dataset collection in gazebo](/img/training_data.gif){: .mx-auto.d-block :}

A few days later, we realized that we may have simplified the problem *too much* - two balls could be on either side of the robot headed directly at it, and all of the values would look the same - because all of the data is relative to the robot, we don't actually have a number that represents where the balls actually are, which is definitely an oversight we should revisit. However, in the interest of getting our data pipeline up and running, we decided to pipe this data into our neural-net anyways to see what behaviors would come up.

<!-- TODO: write more about neural net -->

After some training, it looks like our net did converge to a solution.
![Loss goes down over epoch](/img/initial_test_loss.png){: .mx-auto.d-block :}

How good is this solution? Tune in next time to find out...
