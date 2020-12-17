---
layout: post
title: Simple Neural Networks
subtitle: An attempt at peeking into the black box
---

Looking at the gifs on our [project homepage](/ml_comprobofinal/), it's pretty clear that our robot has attempted to emulate human reflexes. In this blog post, we take a closer look, and try to understand why this behavior came to be. 

It would be difficult to visualize a more complicated neural network with several layers to understand the basis behind the decision making for our model, so in this post, we take a closer look at two different models: one net has only 1 fully connected layer with one output, while the other net has  why. For both of these models, the only input data is the dodgeball's relative position (we'll refer to this as `px` and `py`) and velocity (denoted as `vx` and `vy`) to the robot.

Let's start by taking a look at the results from each model

## The Good
#### 1-Layer
![](/ml_comprobo/standard_990_good.gif)
#### 2-Layer
![](/ml_comprobo/standard_987_good.gif)

## The Bad
#### 1-Layer
![](/ml_comprobo/standard_990_bad.gif)
#### 2-Layer
![](/ml_comprobo/standard_987_bad.gif)


 this is happening, but we can look at the simple case where we just have a 1 layer. This simple net has a similar behavior as the more complicated one, so let’s take a closer look at the output weights

Taking a closer look at our model, it’s clear that it works well when balls come at it from the right or the left, but it struggles when the balls come at it head on
[insert gif here]

It would be difficult to visualize a more complicated neural network with several layers to understand why this is happening, but we can look at the simple case where we just have a 1 layer. This simple net has a similar behavior as the more complicated one, so let’s take a closer look at the output weights
```
[-1.2466,  0.2136,  0.0509, -0.0623, -0.7231]
px         py       vx      vy      bias
```

We can see here that the primary driver is the px value - if the ball comes from right, this net outputs a strong command to drive to the left.

It’s also interesting that this net seemed to learn about human reflexes - it pauses a bit before making a move
[insert gif here]
Interestingly enough, these output weights do also tell that story - between the bias term and the py - py’s average max value was approximately 3, and 0 meant that the robot was about to get hit. It’s worth noting that 3*0.2136 (py weight) approximately equals 0.7231 (magnitude of the bias term), which means that when the ball is far away the bias attempts to negate the effect of the y position. However, as the ball gets closer, the signal becomes more negative, encouraging the robot to move to the left.

Interesting to note the nonlinear behavior
