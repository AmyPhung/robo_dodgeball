---
layout: page
title: Robo Dodgeball
subtitle: (with machine learning)
cover-img: /ml_comprobofinal/img/dodgeball2.jpg
---

## Project Overview
For our final project, we wanted to get more experience with applying machine learning to learn a control policy in robots, so we chose dodgeball as our area of focus due to how scalable it was - we could start with a simple model where a simple differential drive robot learns how to dodge a ball thrown straight at it, and could go as far as implementing this on a drone with balls thrown at it from all directions. Ultimately, we were able to train two different types of models using imitation learning, where we recorded a dataset of us playing dodgeball then trained the model to imitate what we did. Over the course of this project, we looked into 
[Long-Short Term Memory models (LSTM)](https://www.tensorflow.org/api_docs/python/tf/keras/layers/LSTM)
and 
[standard neural networks](https://pytorch.org/tutorials/beginner/blitz/neural_networks_tutorial.html). 
Using these models, we were able to train a differential drive robot to dodge balls with some degree of success. 

#### LSTM:
<img src="/ml_comprobofinal/img/LSTM_08_10_NICE.gif" width="1500"/>

#### Standard:
<img src="/ml_comprobofinal/img/standard_987_good.gif" width="1500"/>

Interestingly enough, in addition to learning how to dodge balls, the models appear to have learned to emulate human reflexes too! 

Curious to read about how we got to this point? Check out our other project webpages!
+ [Machine learning - LSTM](/ml_comprobofinal/LSTM)
+ [Machine learning - Standard Neural Networks](/ml_comprobofinal/neural_network)
+ [System Architecture](/ml_comprobofinal/system_architecture)
+ [Blog (for commentary)](/ml_comprobofinal/blog)
